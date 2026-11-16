`default_nettype none

// Top level. Keep the module name your template already uses
// (it must match tb.v and top_module in info.yaml).
//
// Pins:
//   ui_in[7:0]  data byte to write
//   uio[0]      SCL (open-drain, needs external pull-up)
//   uio[1]      SDA (open-drain, needs external pull-up)
//   uio_in[2]   go: rising edge starts a write
//   uo_out[0]   busy
//   uo_out[1]   nack (target didn't acknowledge)
//   uo_out[2]   done
module tt_um_example (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    // Every input from outside the chip is asynchronous to clk, so each
    // one goes through two flip-flops before the logic uses it.
    reg [1:0] scl_sync, sda_sync;
    reg [2:0] go_sync;   // third flop is for rising-edge detection

    always @(posedge clk) begin
        if (!rst_n) begin
            scl_sync <= 2'b11;   // idle bus reads high
            sda_sync <= 2'b11;
            go_sync  <= 3'b000;
        end else begin
            scl_sync <= {scl_sync[0], uio_in[0]};
            sda_sync <= {sda_sync[0], uio_in[1]};
            go_sync  <= {go_sync[1:0], uio_in[2]};
        end
    end

    wire go_rise = go_sync[1] & ~go_sync[2];

    wire scl_low, sda_low, busy, nack, done;

    i2c_write_byte #(.HALF(300), .ADDR(7'h50)) u_i2c (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (go_rise),
        .data    (ui_in),
        .scl_in  (scl_sync[1]),
        .sda_in  (sda_sync[1]),
        .scl_low (scl_low),
        .sda_low (sda_low),
        .busy    (busy),
        .nack    (nack),
        .done    (done)
    );

    // Open-drain: the output value is always 0. To pull a line low we
    // enable the driver (oe=1); to release it we disable the driver (oe=0)
    // and the pull-up resistor makes the line high.
    assign uio_out = 8'b0;
    assign uio_oe  = {6'b0, sda_low, scl_low};

    assign uo_out  = {5'b0, done, nack, busy};

    wire _unused = &{ena, uio_in[7:3], 1'b0};

endmodule
