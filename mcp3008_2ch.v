// =============================================================
// mcp3008_2ch - doc luan phien 2 kenh cua MCP3008 (SPI mode 0)
//
// Moi lan chuyen doi = 24 xung SCK (khung 3 byte):
//   byte0 = 0000_0001            (bit start o bit cuoi)
//   byte1 = 1 CCC xxxx           (SGL=1, CCC = so kenh)
//   byte2 = xxxx_xxxx
//   ket qua 10 bit = 10 bit cuoi cua 24 bit nhan ve
//
// Ket qua doc luan phien: CH_A, CH_B, CH_A, CH_B, ...
//   adc_valid = 1 xung 1 clk khi co mau moi
//   adc_ch    = 0 -> mau cua CH_A (CH1),  1 -> mau cua CH_B (CH2)
//
// Voi CLK_DIV=25, GAP=50 (clk 50 MHz):
//   SCK = 1 MHz, 1 lan chuyen doi = 1250 clk = 25 us
//   -> moi kenh 1 mau / 50 us (20 ksps moi kenh)
// =============================================================
module mcp3008_2ch #(
    parameter       CLK_DIV = 25,      // nua chu ky SCK, tinh bang clk
    parameter       GAP     = 50,      // CS o muc cao giua 2 lan chuyen doi, tinh bang clk
    parameter [2:0] CH_A    = 3'd0,    // kenh MCP3008 hien thi la CH1
    parameter [2:0] CH_B    = 3'd1     // kenh MCP3008 hien thi la CH2
)(
    input  wire       clk,
    input  wire       rst_n,

    output reg        adc_cs,
    output reg        adc_clk,
    output reg        adc_din,
    input  wire       adc_dout,

    output reg  [9:0] adc_data,
    output reg        adc_valid,
    output reg        adc_ch
);

    localparam S_GAP  = 2'd0;
    localparam S_LOW  = 2'd1;
    localparam S_HIGH = 2'd2;

    reg [1:0]  state;
    reg [7:0]  cnt;
    reg [4:0]  bit_cnt;       // 0..23
    reg [23:0] tx_sh;
    reg [23:0] rx_sh;
    reg        which;         // 0 = CH_A, 1 = CH_B (kenh dang doc)

    // dong bo dau vao DOUT
    reg dout_s0, dout_s;

    always @(posedge clk) begin
        dout_s0 <= adc_dout;
        dout_s  <= dout_s0;
    end


    always @(posedge clk) begin

        if (!rst_n) begin

            state     <= S_GAP;
            cnt       <= 8'd0;
            bit_cnt   <= 5'd0;
            tx_sh     <= 24'd0;
            rx_sh     <= 24'd0;
            which     <= 1'b0;

            adc_cs    <= 1'b1;
            adc_clk   <= 1'b0;
            adc_din   <= 1'b0;
            adc_data  <= 10'd0;
            adc_valid <= 1'b0;
            adc_ch    <= 1'b0;

        end

        else begin

            adc_valid <= 1'b0;

            case (state)

                // ---------------------------------------------
                // CS = 1, cho GAP clk roi bat dau lan chuyen doi moi
                // ---------------------------------------------
                S_GAP: begin

                    adc_cs  <= 1'b1;
                    adc_clk <= 1'b0;

                    if (cnt >= GAP - 1) begin

                        cnt     <= 8'd0;
                        bit_cnt <= 5'd0;

                        tx_sh   <= {8'h01, 1'b1, (which ? CH_B : CH_A), 4'b0000, 8'h00};
                        adc_din <= 1'b0;             // bit 23 = 0

                        adc_cs  <= 1'b0;
                        state   <= S_LOW;

                    end
                    else begin

                        cnt <= cnt + 1'b1;

                    end

                end

                // ---------------------------------------------
                // SCK = 0 : DIN dang giu bit hien tai.
                // Het pha thap -> keo SCK len va lay mau DOUT.
                // ---------------------------------------------
                S_LOW: begin

                    if (cnt >= CLK_DIV - 1) begin

                        cnt     <= 8'd0;
                        adc_clk <= 1'b1;
                        rx_sh   <= {rx_sh[22:0], dout_s};
                        state   <= S_HIGH;

                    end
                    else begin

                        cnt <= cnt + 1'b1;

                    end

                end

                // ---------------------------------------------
                // SCK = 1 : het pha cao -> ha SCK, doi bit DIN
                // ---------------------------------------------
                S_HIGH: begin

                    if (cnt >= CLK_DIV - 1) begin

                        cnt     <= 8'd0;
                        adc_clk <= 1'b0;

                        if (bit_cnt == 5'd23) begin

                            adc_cs    <= 1'b1;
                            adc_data  <= rx_sh[9:0];
                            adc_valid <= 1'b1;
                            adc_ch    <= which;
                            which     <= ~which;
                            state     <= S_GAP;

                        end
                        else begin

                            bit_cnt <= bit_cnt + 1'b1;
                            tx_sh   <= {tx_sh[22:0], 1'b0};
                            adc_din <= tx_sh[22];
                            state   <= S_LOW;

                        end

                    end
                    else begin

                        cnt <= cnt + 1'b1;

                    end

                end

                default: state <= S_GAP;

            endcase

        end

    end

endmodule