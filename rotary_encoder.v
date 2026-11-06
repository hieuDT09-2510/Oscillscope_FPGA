module rotary_encoder (
    input  wire clk,
    input  wire rst_n,

    input  wire enc_a,
    input  wire enc_b,

    // step = 1 xung (1 clk) moi nac xoay, dir = 1 xoay thuan / 0 xoay nguoc.
    // Thay cho "position" 8-bit cu (bi tran 127 -> -128 lam dao chieu).
    output reg              step,
    output reg              dir
);

    // =========================================================
    // Synchronizer
    // =========================================================

    reg a_ff1, a_ff2;
    reg b_ff1, b_ff2;

    // =========================================================
    // Glitch filter cho A/B
    //
    // Chỉ chấp nhận A/B thay đổi khi tín hiệu đã ổn định
    // liên tục trong FILTER_LIMIT chu kỳ clock.
    // Ở 50 MHz, FILTER_LIMIT = 1000 ~ 20 us.
    // Đủ để lọc nhiễu điện, không đủ lâu để làm mất bước
    // khi xoay encoder ở tốc độ bình thường.
    // =========================================================

    localparam FILTER_LIMIT = 16'd1000;

    reg  a_stable, b_stable;

    reg [15:0] a_cnt;
    reg [15:0] b_cnt;


    reg [1:0] prev_state;

    reg signed [2:0] count;


    wire [1:0] current_state;

    assign current_state = {
        a_stable,
        b_stable
    };


    // =========================================================
    // Lọc nhiễu tín hiệu A
    // =========================================================

    always @(posedge clk) begin

        if (!rst_n) begin

            a_ff1 <= 1'b1;
            a_ff2 <= 1'b1;

            a_stable <= 1'b1;
            a_cnt    <= 16'd0;

        end

        else begin

            a_ff1 <= enc_a;
            a_ff2 <= a_ff1;

            if (a_ff2 == a_stable) begin

                a_cnt <= 16'd0;

            end

            else begin

                if (a_cnt >= FILTER_LIMIT) begin

                    a_stable <= a_ff2;
                    a_cnt    <= 16'd0;

                end

                else begin

                    a_cnt <= a_cnt + 1'b1;

                end

            end

        end

    end


    // =========================================================
    // Lọc nhiễu tín hiệu B
    // =========================================================

    always @(posedge clk) begin

        if (!rst_n) begin

            b_ff1 <= 1'b1;
            b_ff2 <= 1'b1;

            b_stable <= 1'b1;
            b_cnt    <= 16'd0;

        end

        else begin

            b_ff1 <= enc_b;
            b_ff2 <= b_ff1;

            if (b_ff2 == b_stable) begin

                b_cnt <= 16'd0;

            end

            else begin

                if (b_cnt >= FILTER_LIMIT) begin

                    b_stable <= b_ff2;
                    b_cnt    <= 16'd0;

                end

                else begin

                    b_cnt <= b_cnt + 1'b1;

                end

            end

        end

    end


    // =========================================================
    // Encoder decoder
    // =========================================================

    always @(posedge clk) begin

        if (!rst_n) begin

            prev_state <= 2'b11;

            count <= 3'sd0;

            step <= 1'b0;

            dir <= 1'b0;

        end

        else begin

            step <= 1'b0;

            // =================================================
            // Detect transition
            // =================================================

            if (current_state != prev_state) begin

                case ({
                    prev_state,
                    current_state
                })

                    // -----------------------------------------
                    // Clockwise
                    // -----------------------------------------

                    4'b0010,
                    4'b1011,
                    4'b1101,
                    4'b0100:

                        count <= count + 1;


                    // -----------------------------------------
                    // Counter-clockwise
                    // -----------------------------------------

                    4'b0001,
                    4'b0111,
                    4'b1110,
                    4'b1000:

                        count <= count - 1;


                    default:

                        count <= count;

                endcase


                // =================================================
                // One complete detent
                // =================================================

                if (count >= 3) begin

                    step <= 1'b1;

                    dir <= 1'b1;

                    count <= 0;

                end


                else if (count <= -3) begin

                    step <= 1'b1;

                    dir <= 1'b0;

                    count <= 0;

                end


                prev_state <= current_state;

            end

        end

    end


endmodule