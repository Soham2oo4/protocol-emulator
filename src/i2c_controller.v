`default_nettype none

// Hardwired I2C controller (reference design, not the final engine).
//
// Supports two transactions on a register-based device (EEPROM, sensor):
//
//   WRITE: START | ADDR+W | ACK | REG | ACK | DATA | ACK | STOP
//   READ:  START | ADDR+W | ACK | REG | ACK | START (repeated) | ADDR+R | ACK |
//          DATA (from target) | NACK | STOP
//
// The controller is built from a few reusable "bus operations"
// (START, TX byte, RX byte, STOP). A small step table (in FETCH) lists
// which operations make up each transaction. That table is the seed of
// the programmable engine: later it becomes a program in memory.
//
// SCL/SDA are open-drain: *_low = 1 pulls the line low, 0 releases it.
module i2c_controller #(
    parameter        HALF = 300,      // clk cycles per half SCL period (60 MHz -> 100 kHz)
    parameter [6:0]  ADDR = 7'h50     // 7-bit target address
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_write,   // 1-cycle pulse: write 'wdata' to register 'reg_addr'
    input  wire       start_read,    // 1-cycle pulse: read register 'reg_addr' into 'rdata'
    input  wire [7:0] reg_addr,
    input  wire [7:0] wdata,
    input  wire       scl_in,        // synchronized SCL level
    input  wire       sda_in,        // synchronized SDA level
    output reg        scl_low,
    output reg        sda_low,
    output wire       busy,
    output reg        nack,          // a byte we sent was not acknowledged
    output reg        done,          // set when a transaction finishes, cleared on start
    output reg  [7:0] rdata          // byte received by the last read
);

    localparam [15:0] HALF_M1 = HALF - 1;
    localparam [15:0] QTR     = HALF / 2;

    // Every bus operation starts and ends with SCL low,
    // except a START from an idle bus and the final STOP.
    localparam [4:0]
        IDLE    = 5'd0,
        FETCH   = 5'd1,    // pick the next operation from the step table
        ST_0    = 5'd2,    // START: release SDA, then release SCL
        ST_1    = 5'd3,    //        SCL high: pull SDA low  (= START)
        ST_2    = 5'd4,    //        pull SCL low
        TX_LOW  = 5'd5,    // TX byte: SCL low, put bit on SDA
        TX_HIGH = 5'd6,    //          SCL high, target reads bit
        TA_LOW  = 5'd7,    // TX ACK:  release SDA for the target
        TA_HIGH = 5'd8,    //          SCL high, read ACK/NACK
        RX_LOW  = 5'd9,    // RX byte: SCL low, keep SDA released
        RX_HIGH = 5'd10,   //          SCL high, sample bit from target
        RA_LOW  = 5'd11,   // RX ACK:  we answer (NACK = release SDA for a single-byte read)
        RA_HIGH = 5'd12,
        SP_0    = 5'd13,   // STOP: SCL low, pull SDA low
        SP_1    = 5'd14,   //       SCL high, release SDA (= STOP)
        SP_2    = 5'd15;   //       bus free time

    reg [4:0]  state;
    reg [15:0] cnt;
    reg [7:0]  shreg;
    reg [2:0]  bit_cnt;
    reg [2:0]  step;       // position in the step table
    reg        is_read;
    reg [7:0]  reg_q, wdata_q;

    assign busy = (state != IDLE);
    wire half_done = (cnt == HALF_M1);

    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= IDLE;
            cnt     <= 16'd0;
            scl_low <= 1'b0;
            sda_low <= 1'b0;
            shreg   <= 8'd0;
            bit_cnt <= 3'd0;
            step    <= 3'd0;
            is_read <= 1'b0;
            reg_q   <= 8'd0;
            wdata_q <= 8'd0;
            nack    <= 1'b0;
            done    <= 1'b0;
            rdata   <= 8'd0;
        end else begin
            cnt <= cnt + 16'd1;   // default; states reset it on transitions

            case (state)
                IDLE: begin
                    scl_low <= 1'b0;
                    sda_low <= 1'b0;
                    cnt     <= 16'd0;
                    if (start_write || start_read) begin
                        is_read <= start_read;
                        reg_q   <= reg_addr;
                        wdata_q <= wdata;
                        step    <= 3'd0;
                        nack    <= 1'b0;
                        done    <= 1'b0;
                        state   <= FETCH;
                    end
                end

                // ---------------- step table ----------------
                FETCH: begin
                    cnt  <= 16'd0;
                    step <= step + 3'd1;
                    case ({is_read, step})
                        // START (write step 0, read steps 0 and 3)
                        4'b0_000, 4'b1_000, 4'b1_011: state <= ST_0;
                        // TX address + W
                        4'b0_001, 4'b1_001: begin shreg <= {ADDR, 1'b0}; bit_cnt <= 3'd7; state <= TX_LOW; end
                        // TX register number
                        4'b0_010, 4'b1_010: begin shreg <= reg_q;        bit_cnt <= 3'd7; state <= TX_LOW; end
                        // TX data (write only)
                        4'b0_011:           begin shreg <= wdata_q;      bit_cnt <= 3'd7; state <= TX_LOW; end
                        // TX address + R (read only)
                        4'b1_100:           begin shreg <= {ADDR, 1'b1}; bit_cnt <= 3'd7; state <= TX_LOW; end
                        // RX one byte (read only)
                        4'b1_101:           begin bit_cnt <= 3'd7;                        state <= RX_LOW; end
                        // STOP (write step 4, read step 6)
                        default:            state <= SP_0;
                    endcase
                end

                // ---------------- START ----------------
                ST_0: begin
                    if (cnt == QTR) sda_low <= 1'b0;          // SDA high first
                    if (half_done) begin
                        scl_low <= 1'b0;                      // then SCL high
                        cnt     <= 16'd0;
                        state   <= ST_1;
                    end
                end

                ST_1: begin
                    if (!scl_in) cnt <= 16'd0;                // wait for SCL to really be high
                    else if (half_done) begin
                        sda_low <= 1'b1;                      // SDA falls while SCL high = START
                        cnt     <= 16'd0;
                        state   <= ST_2;
                    end
                end

                ST_2: if (half_done) begin
                    scl_low <= 1'b1;
                    cnt     <= 16'd0;
                    state   <= FETCH;
                end

                // ---------------- TX byte ----------------
                TX_LOW: begin
                    if (cnt == QTR) sda_low <= ~shreg[7];     // MSB first
                    if (half_done) begin
                        scl_low <= 1'b0;
                        cnt     <= 16'd0;
                        state   <= TX_HIGH;
                    end
                end

                TX_HIGH: begin
                    if (!scl_in) cnt <= 16'd0;                // clock stretching
                    else if (half_done) begin
                        scl_low <= 1'b1;
                        cnt     <= 16'd0;
                        shreg   <= {shreg[6:0], 1'b0};
                        if (bit_cnt == 3'd0) state <= TA_LOW;
                        else begin
                            bit_cnt <= bit_cnt - 3'd1;
                            state   <= TX_LOW;
                        end
                    end
                end

                TA_LOW: begin
                    if (cnt == QTR) sda_low <= 1'b0;          // let the target answer
                    if (half_done) begin
                        scl_low <= 1'b0;
                        cnt     <= 16'd0;
                        state   <= TA_HIGH;
                    end
                end

                TA_HIGH: begin
                    if (!scl_in) cnt <= 16'd0;
                    else begin
                        if (cnt == QTR) nack <= sda_in;       // low = ACK, high = NACK
                        if (half_done) begin
                            scl_low <= 1'b1;
                            cnt     <= 16'd0;
                            state   <= nack ? SP_0 : FETCH;   // NACK: abort with STOP
                        end
                    end
                end

                // ---------------- RX byte ----------------
                RX_LOW: begin
                    if (cnt == QTR) sda_low <= 1'b0;          // keep SDA released: target drives it
                    if (half_done) begin
                        scl_low <= 1'b0;
                        cnt     <= 16'd0;
                        state   <= RX_HIGH;
                    end
                end

                RX_HIGH: begin
                    if (!scl_in) cnt <= 16'd0;
                    else begin
                        if (cnt == QTR) shreg <= {shreg[6:0], sda_in};   // sample mid-high
                        if (half_done) begin
                            scl_low <= 1'b1;
                            cnt     <= 16'd0;
                            if (bit_cnt == 3'd0) state <= RA_LOW;
                            else begin
                                bit_cnt <= bit_cnt - 3'd1;
                                state   <= RX_LOW;
                            end
                        end
                    end
                end

                RA_LOW: begin
                    // Last (only) byte of the read: answer NACK by leaving SDA released.
                    // A multi-byte read would pull SDA low (ACK) for every byte but the last.
                    if (cnt == QTR) sda_low <= 1'b0;
                    if (half_done) begin
                        rdata   <= shreg;
                        scl_low <= 1'b0;
                        cnt     <= 16'd0;
                        state   <= RA_HIGH;
                    end
                end

                RA_HIGH: begin
                    if (!scl_in) cnt <= 16'd0;
                    else if (half_done) begin
                        scl_low <= 1'b1;
                        cnt     <= 16'd0;
                        state   <= FETCH;
                    end
                end

                // ---------------- STOP ----------------
                SP_0: begin
                    if (cnt == QTR) sda_low <= 1'b1;
                    if (half_done) begin
                        scl_low <= 1'b0;
                        cnt     <= 16'd0;
                        state   <= SP_1;
                    end
                end

                SP_1: begin
                    if (!scl_in) cnt <= 16'd0;
                    else if (half_done) begin
                        sda_low <= 1'b0;                      // SDA rises while SCL high = STOP
                        cnt     <= 16'd0;
                        state   <= SP_2;
                    end
                end

                SP_2: if (half_done) begin
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule