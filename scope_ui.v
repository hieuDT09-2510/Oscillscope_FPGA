// =============================================================
// scope_ui - giao dien oscilloscope 2 kenh (CH1 vang, CH2 xanh lo) cho TFT ILI9341
//
// He toa do MAN HINH (sau khi xoay module 90 do de xem):
//   sx = 0..319  (ngang, truc THOI GIAN)  = y_cnt cua tft_ili9341
//   sy = 0..239  (doc,   truc DIEN AP)    = x_cnt cua tft_ili9341
//
// Bo cuc:
//   sy   0..23   : thanh tren  (CH1, CH2, RUN/STOP + TIME/DIV)
//   sy  24..215  : vung song   192 px = 8 div x 24 px
//                              320 px = 10 div x 32 px
//   sy 216..239  : thanh duoi  (V/DIV + AGC)
//
// 'color' tre 3 clk so voi sx/sy. Bo FSM cua tft_ili9341 chi
// lay mau sau ~90 clk ke tu luc x_cnt/y_cnt doi -> du thoi gian.
// =============================================================
module scope_ui (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [8:0]  sx,
    input  wire [8:0]  sy,

    input  wire [3:0]  time_index,   // 0..5 tu EC11 (da chot theo khung)
    input  wire [3:0]  vdiv_index_ch1, // 0..6, V/DIV manual - CH1
    input  wire [3:0]  vdiv_index_ch2, // 0..6, V/DIV manual - CH2
    input  wire        agc_mode_ch1,   // 1 = AGC tu dong, 0 = chinh tay - CH1
    input  wire        agc_mode_ch2,   // 1 = AGC tu dong, 0 = chinh tay - CH2
    input  wire        sel_ch,         // kenh dang duoc num VDIV dieu khien: 0=CH1, 1=CH2
    input  wire [31:0] freq1_hz,       // tan so CH1, don vi Hz
    input  wire [31:0] freq2_hz,       // tan so CH2, don vi Hz
    input  wire        run,            // 1 = RUN, 0 = STOP (THEM MOI)
    input  wire [1:0]  trig_mode,      // 0 = AUTO, 1 = NORMAL, 2 = SINGLE
    input  wire        trig_slope,     // 0 = canh len, 1 = canh xuong
    input  wire        single_done,    // SINGLE da thu xong 1 khung
    input  wire [9:0]  vpp_raw,        // Vpp (don vi ADC) cua kenh dang chon (THEM MOI)
    input  wire [7:0]  trig_y,       // toa do sy tren man hinh ung voi muc trigger

    input  wire [7:0]  trace1_cur,   // CH1: toa do sy cua mau tai cot sx
    input  wire [7:0]  trace1_prev,  // CH1: toa do sy cua mau tai cot sx-1
    input  wire [7:0]  trace2_cur,   // CH2: nhu tren
    input  wire [7:0]  trace2_prev,

    output reg  [15:0] color
);

    // =========================================================
    // BANG MAU RGB565
    // =========================================================

    localparam [15:0] C_BLACK  = 16'h0000;
    localparam [15:0] C_BAR    = 16'h2104;   // xam toi - nen thanh tren/duoi
    localparam [15:0] C_GRID   = 16'h6B4D;   // xam - cham luoi
    localparam [15:0] C_AXIS   = 16'h9CD3;   // xam sang - truc giua + vach chia
    localparam [15:0] C_BORDER = 16'hC618;   // bac - khung
    localparam [15:0] C_LABEL  = 16'hBDF7;   // chu nhan (TIME/DIV, V/DIV...)
    localparam [15:0] C_WHITE  = 16'hFFFF;   // chu gia tri
    localparam [15:0] C_YELLOW = 16'hFFE0;   // chu CH1
    localparam [15:0] C_CYAN   = 16'h07FF;   // chu CH2
    localparam [15:0] C_TRACE1 = 16'hFFE0;   // song CH1 (vang)
    localparam [15:0] C_TRACE2 = 16'h07FF;   // song CH2 (xanh lo)
    localparam [15:0] C_TRIG   = 16'hFC00;   // vach danh dau muc trigger (do)

    localparam [8:0] SCOPE_T = 9'd24;        // hang dau vung song
    localparam [8:0] SCOPE_B = 9'd215;       // hang cuoi vung song


    // =========================================================
    // NHAN TIME/DIV  (6 ky tu)
    //
    //   t/div = 32 cot/div * (samples_per_col / push_repeat) * T_CAP
    //
    // T_CAP = thoi gian giua 2 lan lay mau cua cung 1 kenh
    //       = 2 lan chuyen doi * 25 us = 50 us  (mcp3008_2ch, CLK_DIV=25, GAP=50)
    // >>> Doi CLK_DIV/GAP trong mcp3008_2ch thi phai tinh lai bang nay.
    // =========================================================

    reg [47:0] tdiv;

    always @(*) begin
        case (time_index)
            4'd0:    tdiv = " 200us";   // zoom in x8
            4'd1:    tdiv = " 400us";   // zoom in x4
            4'd2:    tdiv = " 800us";   // zoom in x2
            4'd3:    tdiv = " 1.6ms";   // 1:1
            4'd4:    tdiv = " 3.2ms";   // gop 2
            4'd5:    tdiv = " 6.4ms";   // gop 4
            default: tdiv = "  ??  ";
        endcase
    end


    // =========================================================
    // =========================================================
    // NHAN V/DIV
    //   0 = 100mV/div
    //   1 = 200mV/div
    //   2 = 500mV/div
    //   3 = 1.0V/div
    //   4 = 2.0V/div
    //   5 = 5.0V/div
    //   6 = 10.0V/div
    // =========================================================

    reg [39:0] vdiv1, vdiv2;

    always @(*) begin
        case (vdiv_index_ch1)
            4'd0:    vdiv1 = "100mV";
            4'd1:    vdiv1 = "200mV";
            4'd2:    vdiv1 = "500mV";
            4'd3:    vdiv1 = " 1.0V";
            4'd4:    vdiv1 = " 2.0V";
            4'd5:    vdiv1 = " 5.0V";
            default: vdiv1 = "10.0V";
        endcase
    end

    always @(*) begin
        case (vdiv_index_ch2)
            4'd0:    vdiv2 = "100mV";
            4'd1:    vdiv2 = "200mV";
            4'd2:    vdiv2 = "500mV";
            4'd3:    vdiv2 = " 1.0V";
            4'd4:    vdiv2 = " 2.0V";
            4'd5:    vdiv2 = " 5.0V";
            default: vdiv2 = "10.0V";
        endcase
    end

    // Ky tu mode ("A" = AGC tu dong, "M" = chinh tay) cho tung kenh
    wire [7:0] mode1_ch = agc_mode_ch1 ? "A" : "M";
    wire [7:0] mode2_ch = agc_mode_ch2 ? "A" : "M";


    // =========================================================
    // FONT 5x7  (moi glyph 35 bit = 7 hang x 5 cot, hang tren cung
    // o bit cao nhat, cot trai nhat o bit cao nhat cua moi hang)
    // =========================================================

    function [34:0] font5x7;
        input [7:0] c;
        begin
            case (c)
                "0": font5x7 = 35'b01110_10001_10011_10101_11001_10001_01110;
                "1": font5x7 = 35'b00100_01100_00100_00100_00100_00100_01110;
                "2": font5x7 = 35'b01110_10001_00001_00010_00100_01000_11111;
                "3": font5x7 = 35'b11110_00001_00001_01110_00001_00001_11110;
                "4": font5x7 = 35'b00010_00110_01010_10010_11111_00010_00010;
                "5": font5x7 = 35'b11111_10000_11110_00001_00001_10001_01110;
                "6": font5x7 = 35'b00110_01000_10000_11110_10001_10001_01110;
                "7": font5x7 = 35'b11111_00001_00010_00100_01000_01000_01000;
                "8": font5x7 = 35'b01110_10001_10001_01110_10001_10001_01110;
                "9": font5x7 = 35'b01110_10001_10001_01111_00001_00010_01100;

                "A": font5x7 = 35'b01110_10001_10001_11111_10001_10001_10001;
                "C": font5x7 = 35'b01110_10001_10000_10000_10000_10001_01110;
                "D": font5x7 = 35'b11110_10001_10001_10001_10001_10001_11110;
                "E": font5x7 = 35'b11111_10000_10000_11110_10000_10000_11111;
                "G": font5x7 = 35'b01110_10001_10000_10111_10001_10001_01111;
                "H": font5x7 = 35'b10001_10001_10001_11111_10001_10001_10001;
                "I": font5x7 = 35'b01110_00100_00100_00100_00100_00100_01110;
                "M": font5x7 = 35'b10001_11011_10101_10101_10001_10001_10001;
                "T": font5x7 = 35'b11111_00100_00100_00100_00100_00100_00100;
                "V": font5x7 = 35'b10001_10001_10001_10001_10001_01010_00100;
                "F": font5x7 = 35'b11111_10000_10000_11110_10000_10000_10000;

                // THEM MOI cho chu RUN / STOP
                "R": font5x7 = 35'b11110_10001_10001_11110_10100_10010_10001;
                "U": font5x7 = 35'b10001_10001_10001_10001_10001_10001_01110;
                "N": font5x7 = 35'b10001_11001_10101_10101_10011_10001_10001;
                "S": font5x7 = 35'b01111_10000_10000_01110_00001_00001_11110;
                "O": font5x7 = 35'b01110_10001_10001_10001_10001_10001_01110;
                "P": font5x7 = 35'b11110_10001_10001_11110_10000_10000_10000;

                // THEM MOI cho don vi Hz/kHz va Vpp
                "k": font5x7 = 35'b10000_10000_10010_10100_11000_10100_10010;
                "z": font5x7 = 35'b00000_00000_11111_00010_00100_01000_11111;
                ">": font5x7 = 35'b10000_01000_00100_00010_00100_01000_10000;
                "p": font5x7 = 35'b00000_00000_11110_10001_10001_11110_10000;

                "m": font5x7 = 35'b00000_00000_11010_10101_10101_10101_10101;
                "s": font5x7 = 35'b00000_00000_01111_10000_01110_00001_11110;
                "u": font5x7 = 35'b00000_00000_10001_10001_10001_10011_01101;
                "x": font5x7 = 35'b00000_00000_10001_01010_00100_01010_10001;

                ":": font5x7 = 35'b00000_01100_01100_00000_00000_01100_01100;

                "/": font5x7 = 35'b00001_00010_00010_00100_01000_01000_10000;
                ".": font5x7 = 35'b00000_00000_00000_00000_00000_01100_01100;
                "L": font5x7 = 35'b10000_10000_10000_10000_10000_10000_11111;
                "?": font5x7 = 35'b01110_10001_00001_00010_00100_00000_00100;

                default: font5x7 = 35'b0;   // dau cach va ky tu la
            endcase
        end
    endfunction


    // =========================================================
    // STAGE 1 - tinh o ky tu nao, hang/cot nao cua glyph
    //
    // Chu phong x2: glyph 5x7 -> 10x14 px, o ky tu 12x16 px
    //   thanh tren: chu nam o sy 5..18       (cach le tren 5 px)
    //   thanh duoi: chu nam o sy 221..234
    //   chu bat dau o sx = 4
    // idx = (sx-4)/12 tinh bang (tx*171)>>11 (dung voi tx < 511)
    // =========================================================

    wire in_hdr_txt = (sy >= 9'd5)   && (sy <= 9'd18);
    wire in_ftr_txt = (sy >= 9'd221) && (sy <= 9'd234);
    wire in_ovl_txt = (sy >= 9'd27)  && (sy <= 9'd40);    // Vpp ngay duoi khung tren

    wire [8:0]  ty    = in_hdr_txt ? (sy - 9'd5)   :
                        in_ftr_txt ? (sy - 9'd221) :
                                     (sy - 9'd27);                   // 0..13
    wire        tx_ok = (sx >= 9'd4);
    wire [8:0]  tx    = sx - 9'd4;

    wire [16:0] tx_m  = tx * 8'd171;
    wire [5:0]  c_idx = tx_m[16:11];                                 // 0..26
    wire [8:0]  c_x0  = {c_idx, 3'b000} + {1'b0, c_idx, 2'b00};      // idx * 12
    wire [8:0]  c_off = tx - c_x0;                                   // 0..11


    // =========================================================
    // HIEN THI TAN SO 2 KENH
    //
    // O footer chi con 7 o trong, nen hien thi luan phien:
    //   F1:1000
    //   F2:0500
    // Moi gia tri hien ~0.5 s.
    // =========================================================
    reg [26:0] freq_mux_cnt;   // 27-bit du de dem toi 2s @ 50MHz
    reg        freq_show_ch2;
    reg [15:0] freq1_latch, freq2_latch;   // gia tri "chot" de hien thi (16 bit la du, toi da 20000),
                                            // khong doi giua chung 1 khung
                                            // hinh -> tranh vo/xe chu

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            freq_mux_cnt   <= 27'd0;
            freq_show_ch2  <= 1'b0;
            freq1_latch    <= 16'd0;
            freq2_latch    <= 16'd0;
        end
        else if (freq_mux_cnt == 27'd99_999_999) begin   // 2s @ 50MHz (truoc: 0.5s)
            freq_mux_cnt   <= 27'd0;
            freq_show_ch2  <= ~freq_show_ch2;
            freq1_latch    <= freq1_hz[15:0];   // chot tai thoi diem doi kenh
            freq2_latch    <= freq2_hz[15:0];
        end
        else begin
            freq_mux_cnt <= freq_mux_cnt + 1'b1;
        end
    end

    reg [15:0] freq_disp;   // freq cua kenh dang duoc chon de hien thi

    // Tach chu so tren 14 bit (toi da 9999) thay vi chia/lay du 32 bit:
    // phep chia hang so 32 bit rat ton tai nguyen (LE) tren FPGA nho.
    wire [13:0] fd  = freq_disp[13:0];
    wire [3:0]  dg3 = fd / 14'd1000;          // 0..9
    wire [9:0]  r3  = fd % 14'd1000;
    wire [3:0]  dg2 = r3 / 10'd100;           // 0..9
    wire [6:0]  r2  = r3 % 10'd100;
    wire [3:0]  dg1 = r2 / 7'd10;             // 0..9
    wire [3:0]  dg0 = r2 % 7'd10;             // 0..9

    // 5 o chu (idx 21..25) cho tan so:
    //   f < 1000        : "ddd Hz"  -> vd "500Hz", " 50Hz", "  5Hz"
    //   1000..9999      : "d.ddk"   -> vd "1.00k" (= 1.00 kHz)
    //   f >= 10000      : " >10k"
    reg [7:0] fch0, fch1, fch2, fch3, fch4;

    always @(*) begin
        freq_disp = freq_show_ch2 ? freq2_latch : freq1_latch;

        if (freq_disp < 16'd1000) begin
            fch0 = (dg2 == 4'd0) ? " " : ({4'd0, dg2} + 8'd48);
            fch1 = ((dg2 == 4'd0) && (dg1 == 4'd0)) ? " " : ({4'd0, dg1} + 8'd48);
            fch2 = {4'd0, dg0} + 8'd48;
            fch3 = "H";
            fch4 = "z";
        end
        else if (freq_disp < 16'd10000) begin
            fch0 = {4'd0, dg3} + 8'd48;
            fch1 = ".";
            fch2 = {4'd0, dg2} + 8'd48;
            fch3 = {4'd0, dg1} + 8'd48;
            fch4 = "k";
        end
        else begin
            fch0 = " ";
            fch1 = ">";
            fch2 = "1";
            fch3 = "0";
            fch4 = "k";
        end
    end

    // =========================================================
    // Vpp cua kenh dang chon (sel_ch): mV = range * 3300/1023
    //   ~ range * 3303 / 1024  (dich bit, khong can phep chia)
    // Hien thi "Vpp d.ddV" trong vung song, mau theo kenh.
    // =========================================================
    wire [21:0] vp_m   = vpp_raw * 12'd3303;
    wire [11:0] vpp_mv = vp_m[21:10];             // 0..3302
    wire [3:0]  vp_d0  = vpp_mv / 12'd1000;       // 0..3
    wire [9:0]  vp_r   = vpp_mv % 12'd1000;
    wire [3:0]  vp_d1  = vp_r / 10'd100;
    wire [6:0]  vp_r2  = vp_r % 10'd100;
    wire [3:0]  vp_d2  = vp_r2 / 7'd10;

    // Chu hien thi che do trigger / canh trigger (4 ky tu moi cai)
    reg [31:0] tmode_s;
    reg [31:0] tslope_s;

    always @(*) begin
        case (trig_mode)
            2'd0:    tmode_s = "AUTO";
            2'd1:    tmode_s = "NORM";
            default: tmode_s = "SING";
        endcase

        tslope_s = trig_slope ? "FALL" : "RISE";
    end

    // Ky tu + mau chu tai vi tri hien tai
    //   csel: 0 = chu nhan (xam sang), 1 = chu gia tri (trang), 2 = vang (CH1), 3 = xanh lo (CH2)

    reg [7:0] ch;
    reg [1:0] csel;

    always @(*) begin

        ch   = " ";
        csel = 2'd0;

        if (tx_ok && in_hdr_txt) begin

            // "CH1 CH2 RUN/STOP TIME/DIV 1.6ms"
            case (c_idx)
                6'd0:  begin ch = "C"; csel = 2'd2; end
                6'd1:  begin ch = "H"; csel = 2'd2; end
                6'd2:  begin ch = "1"; csel = 2'd2; end

                6'd4:  begin ch = "C"; csel = 2'd3; end
                6'd5:  begin ch = "H"; csel = 2'd3; end
                6'd6:  begin ch = "2"; csel = 2'd3; end

                // THEM MOI: trang thai RUN / STOP (chu trang)
                6'd7:  begin ch = run ? "R" : "S"; csel = 2'd1; end
                6'd8:  begin ch = run ? "U" : "T"; csel = 2'd1; end
                6'd9:  begin ch = run ? "N" : "O"; csel = 2'd1; end
                6'd10: begin ch = run ? " " : "P"; csel = 2'd1; end

                6'd11: ch = "T";
                6'd12: ch = "I";
                6'd13: ch = "M";
                6'd14: ch = "E";
                6'd15: ch = "/";
                6'd16: ch = "D";
                6'd17: ch = "I";
                6'd18: ch = "V";

                6'd20: begin ch = tdiv[47:40]; csel = 2'd1; end
                6'd21: begin ch = tdiv[39:32]; csel = 2'd1; end
                6'd22: begin ch = tdiv[31:24]; csel = 2'd1; end
                6'd23: begin ch = tdiv[23:16]; csel = 2'd1; end
                6'd24: begin ch = tdiv[15:8];  csel = 2'd1; end
                6'd25: begin ch = tdiv[7:0];   csel = 2'd1; end

                default: ch = " ";
            endcase

        end
        else if (tx_ok && in_ftr_txt) begin

            // "1 100mV A   2 100mV M"
            // Kenh dang duoc num VDIV dieu khien (sel_ch) hien chu
            // TRANG (sang); kenh con lai hien mau nhan (xam) de
            // phan biet nhanh dang chinh kenh nao.
            case (c_idx)
                6'd0:  begin ch = "1"; csel = 2'd2; end   // nhan CH1 (vang)

                6'd2:  begin ch = vdiv1[39:32]; csel = sel_ch ? 2'd0 : 2'd1; end
                6'd3:  begin ch = vdiv1[31:24]; csel = sel_ch ? 2'd0 : 2'd1; end
                6'd4:  begin ch = vdiv1[23:16]; csel = sel_ch ? 2'd0 : 2'd1; end
                6'd5:  begin ch = vdiv1[15:8];  csel = sel_ch ? 2'd0 : 2'd1; end
                6'd6:  begin ch = vdiv1[7:0];   csel = sel_ch ? 2'd0 : 2'd1; end

                6'd8:  begin ch = mode1_ch;     csel = sel_ch ? 2'd0 : 2'd1; end

                6'd11: begin ch = "2"; csel = 2'd3; end   // nhan CH2 (xanh lo)

                6'd13: begin ch = vdiv2[39:32]; csel = sel_ch ? 2'd1 : 2'd0; end
                6'd14: begin ch = vdiv2[31:24]; csel = sel_ch ? 2'd1 : 2'd0; end
                6'd15: begin ch = vdiv2[23:16]; csel = sel_ch ? 2'd1 : 2'd0; end
                6'd16: begin ch = vdiv2[15:8];  csel = sel_ch ? 2'd1 : 2'd0; end
                6'd17: begin ch = vdiv2[7:0];   csel = sel_ch ? 2'd1 : 2'd0; end

                6'd19: begin ch = mode2_ch;     csel = sel_ch ? 2'd1 : 2'd0; end

                // idx 20 de trong lam khoang cach voi "M"; chi con 4 cot
                // (21..24) danh cho gia tri Hz, mau vang=CH1 / cyan=CH2
                // de thay cho nhan "F1:"/"F2:" (tiet kiem cot, khong bi
                // tran ra ngoai canh phai man hinh - sx toi da chi den 319).
                6'd21: begin ch = fch0; csel = freq_show_ch2 ? 2'd3 : 2'd2; end
                6'd22: begin ch = fch1; csel = freq_show_ch2 ? 2'd3 : 2'd2; end
                6'd23: begin ch = fch2; csel = freq_show_ch2 ? 2'd3 : 2'd2; end
                6'd24: begin ch = fch3; csel = freq_show_ch2 ? 2'd3 : 2'd2; end
                6'd25: begin ch = fch4; csel = freq_show_ch2 ? 2'd3 : 2'd2; end

                default: ch = " ";
            endcase

        end
        else if (tx_ok && in_ovl_txt) begin

            // "Vpp d.ddV" - Vpp cua kenh dang chon (vang = CH1, xanh lo = CH2)
            csel = sel_ch ? 2'd3 : 2'd2;

            case (c_idx)
                // Che do trigger (chu trang): "AUTO RISE DONE"
                6'd0:  begin ch = tmode_s[31:24];  csel = 2'd1; end
                6'd1:  begin ch = tmode_s[23:16];  csel = 2'd1; end
                6'd2:  begin ch = tmode_s[15:8];   csel = 2'd1; end
                6'd3:  begin ch = tmode_s[7:0];    csel = 2'd1; end

                6'd5:  begin ch = tslope_s[31:24]; csel = 2'd1; end
                6'd6:  begin ch = tslope_s[23:16]; csel = 2'd1; end
                6'd7:  begin ch = tslope_s[15:8];  csel = 2'd1; end
                6'd8:  begin ch = tslope_s[7:0];   csel = 2'd1; end

                6'd10: begin ch = single_done ? "D" : " "; csel = 2'd1; end
                6'd11: begin ch = single_done ? "O" : " "; csel = 2'd1; end
                6'd12: begin ch = single_done ? "N" : " "; csel = 2'd1; end
                6'd13: begin ch = single_done ? "E" : " "; csel = 2'd1; end

                6'd17: ch = "V";
                6'd18: ch = "p";
                6'd19: ch = "p";

                6'd21: ch = {4'd0, vp_d0} + 8'd48;
                6'd22: ch = ".";
                6'd23: ch = {4'd0, vp_d1} + 8'd48;
                6'd24: ch = {4'd0, vp_d2} + 8'd48;
                6'd25: ch = "V";

                default: ch = " ";
            endcase

        end

    end


    reg [8:0] sx1, sy1;
    reg [7:0] t1c1, t1p1, t2c1, t2p1;
    reg [7:0] ch1;
    reg [2:0] grow1, gcol1;
    reg [1:0] csel1;
    reg       intxt1;

    always @(posedge clk) begin

        sx1    <= sx;
        sy1    <= sy;
        t1c1   <= trace1_cur;
        t1p1   <= trace1_prev;
        t2c1   <= trace2_cur;
        t2p1   <= trace2_prev;

        ch1    <= ch;
        grow1  <= ty[3:1];        // hang glyph 0..6
        gcol1  <= c_off[3:1];     // cot glyph 0..5 (5 = khoang cach)
        csel1  <= csel;
        intxt1 <= tx_ok && (in_hdr_txt || in_ftr_txt || in_ovl_txt);

    end


    // =========================================================
    // STAGE 2 - tra font, tinh cac co (luoi, khung, truc, song)
    // =========================================================

    wire [34:0] glyph = font5x7(ch1);

    reg [4:0] rbits;

    always @(*) begin
        case (grow1)
            3'd0:    rbits = glyph[34:30];
            3'd1:    rbits = glyph[29:25];
            3'd2:    rbits = glyph[24:20];
            3'd3:    rbits = glyph[19:15];
            3'd4:    rbits = glyph[14:10];
            3'd5:    rbits = glyph[9:5];
            3'd6:    rbits = glyph[4:0];
            default: rbits = 5'b00000;
        endcase
    end

    wire txt_px = intxt1 && (gcol1 < 3'd5) && rbits[3'd4 - gcol1];


    wire       in_scope = (sy1 >= SCOPE_T) && (sy1 <= SCOPE_B);
    wire [8:0] ly       = sy1 - SCOPE_T;                 // 0..191 trong vung song

    // vach ngang chinh (bo 0 = khung tren, 96 = truc giua, 192 = khung duoi)
    reg h_major;

    always @(*) begin
        case (ly)
            9'd24, 9'd48, 9'd72, 9'd120, 9'd144, 9'd168: h_major = 1'b1;
            default:                                     h_major = 1'b0;
        endcase
    end

    // khung
    wire f_border = in_scope && ((sx1 == 9'd0) || (sx1 == 9'd319) ||
                                 (sy1 == SCOPE_T) || (sy1 == SCOPE_B));

    // truc giua (net lien) + vach chia nho tren truc
    wire f_axis   = in_scope && ((ly == 9'd96) || (sx1 == 9'd160));

    wire f_tick   = in_scope && (
                        ((sx1[2:0] == 3'd0) && (ly >= 9'd94)  && (ly <= 9'd98)) ||
                        ((ly[2:0]  == 3'd0) && (sx1 >= 9'd158) && (sx1 <= 9'd162)) );

    // luoi cham: 10 div ngang (moi 32 px), 8 div doc (moi 24 px)
    wire f_grid   = in_scope && (
                        ((sx1[4:0] == 5'd0) && (sy1[1:0] == 2'd0)) ||
                        (h_major            && (sx1[1:0] == 2'd0)) );

    // song: noi mau hien tai voi mau truoc (day 2 px)
    wire [7:0] v1_lo = (t1c1 < t1p1) ? t1c1 : t1p1;
    wire [7:0] v1_hi = (t1c1 < t1p1) ? t1p1 : t1c1;
    wire [7:0] v2_lo = (t2c1 < t2p1) ? t2c1 : t2p1;
    wire [7:0] v2_hi = (t2c1 < t2p1) ? t2p1 : t2c1;

    wire f_trace1 = in_scope && (sy1 >= {1'b0, v1_lo}) &&
                                (sy1 <= ({1'b0, v1_hi} + 9'd1));

    wire f_trace2 = in_scope && (sy1 >= {1'b0, v2_lo}) &&
                                (sy1 <= ({1'b0, v2_hi} + 9'd1));

    wire f_bar    = (sy1 < SCOPE_T) || (sy1 > SCOPE_B);

    // vach danh dau muc trigger: doan ngan sat canh trai vung song,
    // tai hang sy = trig_y (da quy doi tu trig_level sang toa do
    // man hinh giong het cach tinh a1_y ben tft_ili9341).
    wire f_trig   = in_scope && (sx1 >= 9'd1) && (sx1 <= 9'd6) &&
                                (sy1 == {1'b0, trig_y});


    reg       r_trace1, r_trace2, r_txt, r_bar, r_border, r_axis, r_grid, r_trig;
    reg [1:0] r_csel;

    always @(posedge clk) begin

        r_trace1 <= f_trace1;
        r_trace2 <= f_trace2;
        r_txt    <= txt_px;
        r_bar    <= f_bar;
        r_border <= f_border;
        r_axis   <= f_axis | f_tick;
        r_grid   <= f_grid;
        r_trig   <= f_trig;
        r_csel   <= csel1;

    end


    // =========================================================
    // STAGE 3 - chon mau theo do uu tien
    //   song CH1 > song CH2 > vach trigger > chu > nen thanh > khung > truc > luoi > nen den
    // =========================================================

    always @(posedge clk) begin

        if      (r_trace1) color <= C_TRACE1;
        else if (r_trace2) color <= C_TRACE2;
        else if (r_trig)   color <= C_TRIG;

        else if (r_txt) begin
            case (r_csel)
                2'd1:    color <= C_WHITE;
                2'd2:    color <= C_YELLOW;
                2'd3:    color <= C_CYAN;
                default: color <= C_LABEL;
            endcase
        end

        else if (r_bar)    color <= C_BAR;
        else if (r_border) color <= C_BORDER;
        else if (r_axis)   color <= C_AXIS;
        else if (r_grid)   color <= C_GRID;
        else               color <= C_BLACK;

    end

endmodule