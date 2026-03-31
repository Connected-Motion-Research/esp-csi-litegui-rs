//! Display rendering task for CSI heatmap visualization.

use embassy_executor::task;
use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, with_timeout};
use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use profont::{PROFONT_18_POINT, PROFONT_24_POINT};
use rm690b0_rs::{Lgt4s3Driver, Rm690b0Driver};

use crate::{
    config::{
        DISPLAY_SIZE, DisplayMode, DisplayResetDriver, HEATMAP_EFFECTIVE_HEIGHT, HEATMAP_EFFECTIVE_WIDTH,
        HEATMAP_START_X, HEATMAP_START_Y, VALID_SUBCARRIER_COUNT,
    },
    state::{CSI_FRAMES, MODE_INTENTS, ModeIntent},
};

#[task]
/// Background task that renders CSI frames to the display framebuffer and flushes updates.
pub async fn display_task(
    mut display_driver: Rm690b0Driver<Lgt4s3Driver, DisplayResetDriver, Rgb888>,
) {
    // Initial full-screen clear
    display_driver.clear(Rgb888::new(0, 0, 0)).unwrap();

    // Text styles
    let title_text_style = MonoTextStyle::new(&PROFONT_24_POINT, Rgb888::YELLOW);
    let static_text_style = MonoTextStyle::new(&PROFONT_18_POINT, Rgb888::WHITE);
    let clear_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb888::new(0, 0, 0))
        .build();

    // Static element drawing in framebuffer
    // Draw static y-axis label vertically, character-by-character
    let yaxis_label = "Subcarriers";
    let yaxis_label_style = static_text_style.clone();
    let char_height = yaxis_label_style.font.character_size.height as i32;
    let char_width = yaxis_label_style.font.character_size.width as i32;
    let yaxis_label_height = yaxis_label.chars().count() as i32 * char_height;

    // Calculate position to be centered vertically next to the heatmap
    let label_start_y =
        HEATMAP_START_Y as i32 + (HEATMAP_EFFECTIVE_HEIGHT as i32 - yaxis_label_height) / 2;
    let label_x = (HEATMAP_START_X as i32 - char_width - 5).max(0); // 5px padding

    // Loop and draw each character indivudually
    for (i, ch) in yaxis_label.chars().enumerate() {
        let mut buffer = [0u8; 4];
        let char_str = ch.encode_utf8(&mut buffer);
        Text::new(
            char_str,
            Point::new(label_x, label_start_y + (i as i32 * char_height)),
            yaxis_label_style.clone(),
        )
        .draw(&mut display_driver)
        .unwrap();
    }

    // Draw static x-axis label horizontally & center
    let xaxis_label_str = "Packets";
    let xaxis_label = Text::new(xaxis_label_str, Point::zero(), static_text_style.clone());
    let xaxis_label_width = xaxis_label.bounding_box().size.width as i32;
    let xaxis_label_x =
        HEATMAP_START_X as i32 + (HEATMAP_EFFECTIVE_WIDTH as i32 - xaxis_label_width) / 2;
    let padding = 10i32; // Text padding from bottom of heatmap
    let xaxis_label_y = (HEATMAP_START_Y + HEATMAP_EFFECTIVE_HEIGHT) as i32
        + padding
        + static_text_style.font.baseline as i32;
    Text::new(
        xaxis_label_str,
        Point::new(xaxis_label_x, xaxis_label_y),
        static_text_style.clone(),
    )
    .draw(&mut display_driver)
    .unwrap();

    let mut current_display_mode = DisplayMode::Magnitude;
    let mut desired_display_mode = DisplayMode::Magnitude;
    let mut last_mode_apply: Option<Instant> = None;
    const MODE_APPLY_MIN_INTERVAL: Duration = Duration::from_millis(450);

    let text_strip_height: i32 = 40;
    let text_strip_y: i32 = (HEATMAP_START_Y as i32 - text_strip_height).max(0);

    // Draw title text
    let draw_title = |display: &mut Rm690b0Driver<_, _, _>, mode: DisplayMode| {
        let mode_str = match mode {
            DisplayMode::Magnitude => "Magnitude",
            DisplayMode::Phase => "Phase",
        };
        let title_text = Text::new(mode_str, Point::zero(), title_text_style.clone());
        let text_width = title_text.bounding_box().size.width as i32;
        let centered_x = HEATMAP_START_X as i32 + (HEATMAP_EFFECTIVE_WIDTH as i32 - text_width) / 2;

        Rectangle::new(
            Point::new(HEATMAP_START_X as i32, text_strip_y),
            Size::new(HEATMAP_EFFECTIVE_WIDTH, text_strip_height as u32),
        )
        .into_styled(clear_style)
        .draw(display)
        .unwrap();

        Text::new(
            mode_str,
            Point::new(centered_x, HEATMAP_START_Y as i32 - 20),
            title_text_style.clone(),
        )
        .draw(display)
        .ok();
    };

    draw_title(&mut display_driver, current_display_mode);

    // Flush static text changes
    display_driver
        .partial_flush(
            0,
            (DISPLAY_SIZE.width - 1) as u16,
            0,
            (DISPLAY_SIZE.height - 1) as u16
        )
        .unwrap();

    // column_width needs to be an even number for proper wrapping
    // Otherwise the last column will be thinner than the rest and heatmap wont scale properly.
    let column_width: u32 = 2;
    #[cfg(feature = "lilygo-t4")]
    let max_columns_per_cycle: u32 = 12;
    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    let max_columns_per_cycle: u32 = 1;
    let col_count = HEATMAP_EFFECTIVE_WIDTH / column_width;
    let mut current_col: u32 = 0;
    let total_height: u32 = HEATMAP_EFFECTIVE_HEIGHT;
    let base_row_height: u32 = total_height / VALID_SUBCARRIER_COUNT as u32;
    let extra_rows: u32 = total_height % VALID_SUBCARRIER_COUNT as u32;

    loop {
        while let Ok(intent) = MODE_INTENTS.try_receive() {
            desired_display_mode = match intent {
                ModeIntent::Next => desired_display_mode.next(),
                ModeIntent::Previous => desired_display_mode.previous(),
            };
        }

        if desired_display_mode != current_display_mode {
            let interval_ok = last_mode_apply
                .map(|t| Instant::now().saturating_duration_since(t) >= MODE_APPLY_MIN_INTERVAL)
                .unwrap_or(true);

            if interval_ok {
                current_display_mode = desired_display_mode;
                last_mode_apply = Some(Instant::now());

                draw_title(&mut display_driver, current_display_mode);

                Rectangle::new(
                    Point::new(HEATMAP_START_X as i32, HEATMAP_START_Y as i32),
                    Size::new(HEATMAP_EFFECTIVE_WIDTH, HEATMAP_EFFECTIVE_HEIGHT),
                )
                .into_styled(clear_style)
                .draw(&mut display_driver)
                .ok();

                current_col = 0;

                display_driver
                    .partial_flush(
                        HEATMAP_START_X as u16,
                        (HEATMAP_START_X + HEATMAP_EFFECTIVE_WIDTH - 1) as u16,
                        text_strip_y as u16,
                        (HEATMAP_START_Y + HEATMAP_EFFECTIVE_HEIGHT - 1) as u16
                    )
                    .ok();
            }
        }

        let first_frame = match with_timeout(Duration::from_millis(20), CSI_FRAMES.receive()).await {
            Ok(frame) => Some(frame),
            Err(_) => None,
        };

        let mut next_frame = match first_frame {
            Some(frame) => Some(frame),
            None => continue,
        };
        let start_col = current_col;
        let mut rendered_cols = 0u32;

        while rendered_cols < max_columns_per_cycle {
            let csi_frame = if let Some(frame) = next_frame.take() {
                frame
            } else {
                match CSI_FRAMES.try_receive() {
                    Ok(frame) => frame,
                    Err(_) => break,
                }
            };

            let csi_data = match current_display_mode {
                DisplayMode::Magnitude => csi_frame.magnitude,
                DisplayMode::Phase => csi_frame.phase,
            };
            let x = HEATMAP_START_X + current_col * column_width;
            let square_width = if current_col == col_count - 1 {
                HEATMAP_EFFECTIVE_WIDTH - (current_col * column_width)
            } else {
                column_width
            };

            if square_width == 0 {
                current_col = (current_col + 1) % col_count;
                rendered_cols += 1;
                continue;
            }

            let mut y_pos: u32 = HEATMAP_START_Y;

            for row in 0..VALID_SUBCARRIER_COUNT {
                let normalized_data = csi_data[row];
                let color = Rgb888::new(
                    (normalized_data * 255.0) as u8,
                    ((1.0 - (2.0 * (normalized_data - 0.5).abs())) * 255.0) as u8,
                    ((1.0 - normalized_data) * 255.0) as u8,
                );

                let square_height =
                    base_row_height + if (row as u32) < extra_rows { 1 } else { 0 };

                if square_height > 0 {
                    Rectangle::new(
                        Point::new(x as i32, y_pos as i32),
                        Size::new(square_width, square_height),
                    )
                    .into_styled(PrimitiveStyleBuilder::new().fill_color(color).build())
                    .draw(&mut display_driver)
                    .ok();
                }

                y_pos += square_height;
            }

            current_col = (current_col + 1) % col_count;
            rendered_cols += 1;

            // Keep gesture task responsive on the same executor core.
            yield_now().await;
        }

        if rendered_cols == 0 {
            continue;
        }

        let mut flush_column_range = |col_start: u32, col_end_exclusive: u32| {
            if col_start >= col_end_exclusive {
                return;
            }

            let x_start_base = HEATMAP_START_X + col_start * column_width;
            let x_end_base = HEATMAP_START_X + col_end_exclusive * column_width - 1;
            let y_start_base = HEATMAP_START_Y;
            let y_end_base = HEATMAP_START_Y + HEATMAP_EFFECTIVE_HEIGHT - 1;

            #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
            {
                // RM690B0 requires x/y starts and window width/height to be even.
                let mut x_start = x_start_base;
                let mut x_end = x_end_base;
                let mut y_start = y_start_base;
                let mut y_end = y_end_base;

                x_start &= !1;
                y_start &= !1;

                let width = x_end.saturating_sub(x_start).saturating_add(1);
                let height = y_end.saturating_sub(y_start).saturating_add(1);
                let even_width = width & !1;
                let even_height = height & !1;

                if even_width == 0 || even_height == 0 {
                    return;
                }

                x_end = x_start + even_width - 1;
                y_end = y_start + even_height - 1;

                let _ = display_driver.partial_flush(
                    x_start as u16,
                    x_end as u16,
                    y_start as u16,
                    y_end as u16,
                    ColorMode::Rgb888,
                );
                return;
            }

            #[cfg(feature = "lilygo-t4")]
            {
                let _ = display_driver.partial_flush(
                    x_start_base as u16,
                    x_end_base as u16,
                    y_start_base as u16,
                    y_end_base as u16,
                );
            }
        };

        // Same wrap strategy as lilygo path, with board-specific window normalization.
        if start_col < current_col {
            flush_column_range(start_col, current_col);
        } else if start_col > current_col {
            flush_column_range(start_col, col_count);
            flush_column_range(0, current_col);
        }

        // Additional fairness point for other tasks (e.g., touch handling).
        yield_now().await;
    }
}
