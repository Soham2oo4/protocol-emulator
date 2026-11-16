`default_nettype none

// Hardwired I2C controller (reference design, not the final engine).
// On a 'start' pulse it performs one write transaction:
//   START | ADDR(7) + W(0) | ACK | DATA(8) | ACK | STOP
// SCL and SDA are open-drain: we either pull the line low or release it
// and let the external pull-up resistor make it high.
module i2c_write_byte #(
    parameter        HALF = 300,      // clk cycles per half SCL period (60 MHz -> 100 kHz)
    parameter [6:0]  ADDR = 7'h50     // 7-bit target address
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,    // 1-cycle pulse: begin a transaction
    input  wire [7:0] data,     // byte to write
    input  wire       scl_in,   // synchronized SCL line level
    input  wire       sda_in,   // synchronized SDA line level
    output reg        scl_low,  // 1 = pull SCL low, 0 = release
    output reg        sda_low,  // 1 = pull SDA low, 0 = release
    output wire       busy,
    output reg        nack,     // 1 = target did not acknowledge
    output reg        done      // set when a transaction finishes, cleared on start
);

    localparam [15:0] HALF_M1 = HALF - 1;
    localparam [15:0] QTR     = HALF / 2;

    localparam [3:0] IDLE     = 4'd0,
                     START_A  = 4'd1,   // bus idle before START
                     START_B  = 4'd2,   // SDA low while SCL high = START
                     BIT_LOW  = 4'd3,   // SCL low: put next data bit on SDA
                     BIT_HIGH = 4'd4,   // SCL high: target reads the bit
                     ACK_LOW  = 4'd5,   // SCL low: release SDA for the target
                     ACK_HIGH = 4'd6,   // SCL high: read the ACK bit
                     STOP_A   = 4'd7,   // SCL low: pull SDA low
                     STOP_B   = 4'd8,   // SCL high, then release SDA = STOP
                     STOP_C   = 4'd9;   // bus free time

    reg [3:0]  state;
    reg [15:0] cnt;       // cycles spent in the current half-period
    reg [7:0]  shreg;     // byte being sent, MSB first
    reg [7:0]  data_q;    // latched data byte
    reg [2:0]  bit_cnt;   // bits left in the current byte
    reg        byte_idx;  // 0 = address byte, 1 = data byte

    assign busy = (state != IDLE);
    wire half_done = (cnt == HALF_M1);

    always @(posedge clk) begin
        if (!rst_n) begin
            state    <= IDLE;
            cnt      <= 16'd0;
            scl_low  <= 1'b0;
            sda_low  <= 1'b0;
            shreg    <= 8'd0;
            data_q   <= 8'd0;
            bit_cnt  <= 3'd0;
            byte_idx <= 1'b0;
            nack     <= 1'b0;
            done     <= 1'b0;
        end else begin
            cnt <= cnt + 16'd1;   // default; states reset it on transitions

            case (state)
                IDLE: begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    cnt     <= 16'd0;
                    if (start) begin
                        shreg    <= {ADDR, 1'b0};   // address + write bit
                        data_q   <= data;
                        byte_idx <= 1'b0;
                        nack     <= 1'b0;
                        done     <= 1'b0;
                        state    <= START_A;
                    end
                end

                START_A: if (half_done) begin
                    sda_low <= 1'b1;                // SDA falls while SCL high
                    cnt     <= 16'd0;
                    state   <= START_B;
                end

                START_B: if (half_done) begin
                    scl_low <= 1'b1;
                    bit_cnt <= 3'd7;
                    cnt     <= 16'd0;
                    state   <= BIT_LOW;
                end

                BIT_LOW: begin
                    if (cnt == QTR) sda_low <= ~shreg[7];   // change SDA only while SCL is low
                    if (half_done) begin
                        scl_low <= 1'b0;                    // release SCL
                        cnt     <= 16'd0;
                        state   <= BIT_HIGH;
                    end
                end

                BIT_HIGH: begin
                    if (!scl_in) begin
                        cnt <= 16'd0;    // SCL not high yet: target is stretching the clock
                    end else if (half_done) begin
                        scl_low <= 1'b1;
                        cnt     <= 16'd0;
                        shreg   <= {shreg[6:0], 1'b0};
                        if (bit_cnt == 3'd0)
                            state <= ACK_LOW;
                        else begin
                            bit_cnt <= bit_cnt - 3'd1;
                            state   <= BIT_LOW;
                        end
                    end
                end

                ACK_LOW: begin
                    if (cnt == QTR) sda_low <= 1'b0;        // let the target drive SDA
                    if (half_done) begin
                        scl_low <= 1'b0;
                        cnt     <= 16'd0;
                        state   <= ACK_HIGH;
                    end
                end

                ACK_HIGH: begin
                    if (!scl_in) begin
                        cnt <= 16'd0;    // clock stretching
                    end else begin
                        if (cnt == QTR) nack <= sda_in;     // low = ACK, high = NACK
                        if (half_done) begin
                            scl_low <= 1'b1;
                            cnt     <= 16'd0;
                            if (nack || byte_idx)
                                state <= STOP_A;
                            else begin
                                byte_idx <= 1'b1;
                                shreg    <= data_q;
                                bit_cnt  <= 3'd7;
                                state    <= BIT_LOW;
                            end
                        end
                    end
                end

                STOP_A: begin
                    if (cnt == QTR) sda_low <= 1'b1;
                    if (half_done) begin
                        scl_low <= 1'b0;
                        cnt     <= 16'd0;
                        state   <= STOP_B;
                    end
                end

                STOP_B: begin
                    if (!scl_in) begin
                        cnt <= 16'd0;
                    end else if (half_done) begin
                        sda_low <= 1'b0;                    // SDA rises while SCL high = STOP
                        cnt     <= 16'd0;
                        state   <= STOP_C;
                    end
                end

                STOP_C: if (half_done) begin
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
