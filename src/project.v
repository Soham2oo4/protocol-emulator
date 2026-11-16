`default_nettype none

// Top level. Keep the module name your template already uses
// (it must match tb.v and top_module in info.yaml).
//
// Pins:
//   ui_in[7:0]   byte in: register number (with set_reg) or data to write
//   uo_out[7:0]  byte read by the last read transaction
//   uio[0]       SCL (open-drain, needs external pull-up)
//   uio[1]       SDA (open-drain, needs external pull-up)
//   uio_in[2]    go_write: rising edge writes ui_in to the selected register
//   uio_in[3]    set_reg:  rising edge latches ui_in as the register number
//   uio_in[4]    go_read:  rising edge reads the selected register
//   uio_out[5]   busy
//   uio_out[6]   nack (target didn't acknowledge)
//   uio_out[7]   done
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

    // Two-flop synchronizers for every asynchronous input we use.
    // Control inputs get a third flop for rising-edge detection.
    reg [1:0] scl_sync, sda_sync;
    reg [2:0] wr_sync, reg_sync, rd_sync;

    always @(posedge clk) begin
        if (!rst_n) begin
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
            wr_sync  <= 3'b000;
            reg_sync <= 3'b000;
            rd_sync  <= 3'b000;
        end else begin
            scl_sync <= {scl_sync[0], uio_in[0]};
            sda_sync <= {sda_sync[0], uio_in[1]};
            wr_sync  <= {wr_sync[1:0],  uio_in[2]};
            reg_sync <= {reg_sync[1:0], uio_in[3]};
            rd_sync  <= {rd_sync[1:0],  uio_in[4]};
        end
    end

    wire go_write = wr_sync[1]  & ~wr_sync[2];
    wire set_reg  = reg_sync[1] & ~reg_sync[2];
    wire go_read  = rd_sync[1]  & ~rd_sync[2];

    // Register number to access, loaded from ui_in on set_reg
    reg [7:0] reg_addr;
    always @(posedge clk) begin
        if (!rst_n)       reg_addr <= 8'd0;
        else if (set_reg) reg_addr <= ui_in;
    end

    wire scl_low, sda_low, busy, nack, done;
    wire [7:0] rdata;

    i2c_controller #(.HALF(300), .ADDR(7'h50)) u_i2c (
        .clk         (clk),
        .rst_n       (rst_n),
        .start_write (go_write),
        .start_read  (go_read),
        .reg_addr    (reg_addr),
        .wdata       (ui_in),
        .scl_in      (scl_sync[1]),
        .sda_in      (sda_sync[1]),
        .scl_low     (scl_low),
        .sda_low     (sda_low),
        .busy        (busy),
        .nack        (nack),
        .done        (done),
        .rdata       (rdata)
    );

    assign uo_out = rdata;

    // uio[0], uio[1]: open-drain (output value 0, enable = pull low)
    // uio[2..4]:      inputs
    // uio[5..7]:      status outputs
    assign uio_out = {done, nack, busy, 5'b00000};
    assign uio_oe  = {3'b111, 3'b000, sda_low, scl_low};

    wire _unused = &{ena, uio_in[7:5], 1'b0};

endmodule