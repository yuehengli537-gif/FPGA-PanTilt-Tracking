// Camera tracking + desktop UART feed ONE 27 MHz servo controller/PWM engine.
// video_* belong to video_clk. The wrapper transfers an atomic snapshot and
// publishes invalid immediately when video_source_alive falls. If video_clk
// itself stops, servo_uart_control's measurement freshness timeout freezes auto.
module camera_servo_uart #(
    parameter integer CLK_HZ=27000000,
    parameter integer BAUD=115200,
    parameter integer WATCHDOG_MS=1000,
    parameter integer RX_TIMEOUT_MS=20,
    parameter integer AUTO_PAN_REVERSE=0,
    parameter integer AUTO_TILT_REVERSE=0
)(
    input wire clk,rst_n,
    input wire video_clk,video_source_alive,
    input wire video_frame_update,video_target_valid,
    input wire [1:0] video_track_state,
    input wire signed [12:0] video_error_x,video_error_y,
    input wire uart_rx,
    output wire uart_tx,servo_pan_pwm,servo_tilt_pwm,
    output wire enabled,auto_mode,timeout_hold,moving,
    output wire [11:0] current_pan_us,current_tilt_us,
    output wire [11:0] target_pan_us,target_tilt_us
);
    (* syn_preserve=1, async_reg="true" *) reg [1:0] control_reset_sync=2'b00;
    (* syn_preserve=1, async_reg="true" *) reg [1:0] video_reset_sync=2'b00;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)control_reset_sync<=0;
        else control_reset_sync<={control_reset_sync[0],1'b1};
    always @(posedge video_clk or negedge rst_n)
        if(!rst_n)video_reset_sync<=0;
        else video_reset_sync<={video_reset_sync[0],1'b1};
    wire control_run_n=rst_n && control_reset_sync[1];
    wire video_run_n=rst_n && video_reset_sync[1];
    reg video_alive_previous;
    always @(posedge video_clk or negedge video_run_n)
        if(!video_run_n)video_alive_previous<=0;
        else video_alive_previous<=video_source_alive;

    wire video_invalidate=video_alive_previous && !video_source_alive;
    wire video_valid=video_source_alive && video_target_valid;
    wire [28:0] video_measurement={video_valid,
        video_valid ? video_track_state : 2'b00,
        video_valid ? video_error_x : 13'd0,
        video_valid ? video_error_y : 13'd0};
    wire measurement_update;
    wire [28:0] measurement;
    servo_measurement_cdc #(.WIDTH(29)) measurement_bridge(
        .arst_n(rst_n),.src_clk(video_clk),
        .src_update(video_frame_update || video_invalidate),
        .src_data(video_measurement),.dst_clk(clk),
        .dst_update(measurement_update),.dst_data(measurement));

    wire [7:0] rx_data,tx_data;
    wire rx_valid,rx_error,tx_valid,tx_ready;
    uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) receiver(
        .clk(clk),.rst_n(control_run_n),.rx(uart_rx),.data(rx_data),
        .valid(rx_valid),.framing_error(rx_error));
    uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) transmitter(
        .clk(clk),.rst_n(control_run_n),.data(tx_data),.valid(tx_valid),
        .ready(tx_ready),.tx(uart_tx));
    servo_uart_control #(
        .CLK_HZ(CLK_HZ),.WATCHDOG_MS(WATCHDOG_MS),
        .RX_TIMEOUT_MS(RX_TIMEOUT_MS),.AUTO_SUPPORT(1),
        .AUTO_PAN_REVERSE(AUTO_PAN_REVERSE),.AUTO_TILT_REVERSE(AUTO_TILT_REVERSE)
    ) control(
        .clk(clk),.rst_n(control_run_n),.rx_data(rx_data),.rx_valid(rx_valid),
        .rx_error(rx_error),.tx_data(tx_data),.tx_valid(tx_valid),.tx_ready(tx_ready),
        .measurement_update(measurement_update),.target_valid(measurement[28]),
        .track_state(measurement[27:26]),
        .error_x($signed(measurement[25:13])),.error_y($signed(measurement[12:0])),
        .servo_pan_pwm(servo_pan_pwm),.servo_tilt_pwm(servo_tilt_pwm),
        .enabled(enabled),.auto_mode(auto_mode),.moving(moving),.timeout_hold(timeout_hold),
        .current_pan_us(current_pan_us),.current_tilt_us(current_tilt_us),
        .target_pan_us(target_pan_us),.target_tilt_us(target_tilt_us));
endmodule
