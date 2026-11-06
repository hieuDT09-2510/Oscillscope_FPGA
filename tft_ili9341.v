module tft_ili9341 (
    input  wire clk,
    input  wire rst_n,

    output reg TFT_CS,
    output reg TFT_DC,
    output reg TFT_RST,
    output reg TFT_SCK,
    output reg TFT_MOSI,

    output reg ready,

    // =========================================================
    // WAVEFORM INPUT
    // =========================================================
    input  wire       wave_push,
    input  wire [9:0] wave_sample,     // CH1
    input  wire [9:0] wave_sample2,    // CH2

    // =========================================================
    // TIME/DIV tu EC11 (top.v: time_index) - MOI THEM
    // =========================================================
    input  wire [3:0] time_index,

    // =========================================================
    // V/DIV tu EC11 thu 2 (top.v: vdiv_index_chX / agc_mode_chX)
    // RIENG CHO TUNG KENH:
    //   agc_mode_chX = 1 : tu dong can bien do (AGC) cho kenh X
    //   agc_mode_chX = 0 : chinh tay, dung vdiv_index_chX (0..6 = x1..x64)
    //   sel_ch         : kenh dang duoc num VDIV dieu khien (0=CH1,1=CH2)
    //                    - chi dung de hien thi tren scope_ui
    // =========================================================
    input  wire [3:0] vdiv_index_ch1,
    input  wire [3:0] vdiv_index_ch2,
    input  wire       agc_mode_ch1,
    input  wire       agc_mode_ch2,
    input  wire       sel_ch,
    input  wire [31:0] freq1_hz,
    input  wire [31:0] freq2_hz,

    // =========================================================
    // RUN/STOP (THEM MOI): 1 = RUN, 0 = STOP (dong bang)
    // =========================================================
    input  wire        run,

    // Che do trigger (chi de hien thi tren scope_ui)
    input  wire [1:0]  trig_mode,    // 0 = AUTO, 1 = NORMAL, 2 = SINGLE
    input  wire        trig_slope,   // 0 = canh len, 1 = canh xuong
    input  wire        single_done,  // SINGLE da thu xong 1 khung

    // =========================================================
    // VERTICAL POSITION (THEM MOI): do lech Y (pixel) cho tung kenh,
    // cong luc HIEN THI. + = song di xuong, - = song di len.
    // =========================================================
    input  wire signed [7:0] off_ch1,
    input  wire signed [7:0] off_ch2,

    // =========================================================
    // TRIGGER (THEM MOI) - cho top.v
    //   cap_active : 1 = dang cho trigger / dang nap 320 cot
    //   trig_level : muc trigger (don vi ADC) = (min+max)/2 khung truoc
    //   trig_hys   : hysteresis
    //   trig_valid : 1 = bien do du lon de trigger
    // =========================================================
    output wire        cap_active,
    output wire [9:0]  trig_level,
    output wire [9:0]  trig_hys,
    output wire        trig_valid
);

    parameter SPI_DIV = 2;   // 2 -> SCK ~12.5 MHz (neu nhieu/sai mau thi doi 3; 6 la ban cu ~4 MHz)

    reg [7:0] state;

    localparam ST_RESET       = 8'd0;
    localparam ST_RESET_WAIT  = 8'd1;
    localparam ST_SWRESET     = 8'd2;
    localparam ST_SWRESET_WAIT= 8'd3;
    localparam ST_SLEEP       = 8'd4;
    localparam ST_SLEEP_WAIT  = 8'd5;
    localparam ST_PIXFMT_CMD  = 8'd6;
    localparam ST_PIXFMT_DATA = 8'd7;
    localparam ST_MADCTL_CMD  = 8'd8;
    localparam ST_MADCTL_DATA = 8'd9;
    localparam ST_DISP_CMD    = 8'd10;
    localparam ST_DISP_WAIT   = 8'd11;
    localparam ST_CASET_CMD   = 8'd12;
    localparam ST_CASET_1     = 8'd13;
    localparam ST_CASET_2     = 8'd14;
    localparam ST_CASET_3     = 8'd15;
    localparam ST_CASET_4     = 8'd16;
    localparam ST_PASET_CMD   = 8'd17;
    localparam ST_PASET_1     = 8'd18;
    localparam ST_PASET_2     = 8'd19;
    localparam ST_PASET_3     = 8'd20;
    localparam ST_PASET_4     = 8'd21;
    localparam ST_RAMWR_CMD   = 8'd22;

    localparam ST_PIXEL_HIGH  = 8'd23;
    localparam ST_PIXEL_LOW   = 8'd24;

    reg [7:0] spi_shift;
    reg [3:0] spi_bit;
    reg [7:0] spi_div_cnt;
    reg spi_busy;

    reg [7:0] tx_byte;
    reg tx_request;
    reg spi_done;

    reg [22:0] wait_cnt;

    // =========================================================
    // GIU NGUYEN DIA CHI VAT LY NHU BAN GOC (240 x 320, MADCTL
    // 0x48) - day la cau hinh DA CHAY DUNG, khong dong vao nua.
    // "Ngang" duoc tao ra bang cach xoay NOI DUNG hinh anh, ban
    // xoay VAT LY module 90 do de xem cho dung chieu.
    // =========================================================

    reg [8:0] x_cnt;   // 0 .. 239 (CASET - truc ngan, vat ly)  = sy tren man hinh da xoay
    reg [8:0] y_cnt;   // 0 .. 319 (PASET - truc dai, vat ly)   = sx tren man hinh da xoay

    // =========================================================
    // WAVEFORM BUFFER - 2 KENH
    //
    // Chi so = y_cnt (0..319, truc thoi gian). Gia tri = toa do sy
    // tren man hinh trong vung song cua scope_ui: 24 (dinh) .. 215
    // (day). 0V = 215.
    //
    // Viet theo kieu doc dong bo (1 clk tre) de Quartus dua vao
    // block RAM M9K thay vi 2 x 2560 flip-flop + mux khong lo.
    // Do FSM chi lay mau sau ~90 clk ke tu luc x_cnt/y_cnt doi
    // nen do tre nay khong anh huong gi.
    // =========================================================

    reg [7:0] wave_buf1 [0:319];   // CH1
    reg [7:0] wave_buf2 [0:319];   // CH2
    reg [8:0] row_ptr;
    reg       capture_active;      // 1 = duoc phep ghi vao wave_buf

    integer wi;

    initial begin
        for (wi = 0; wi < 320; wi = wi + 1) begin
            wave_buf1[wi] = 8'd215;    // day vung song = 0V
            wave_buf2[wi] = 8'd215;
        end
    end

    // =========================================================
    // AUTO-SCALE (AGC) BIEN DO - RIENG CHO TUNG KENH
    //
    // Moi kenh tu do min/max cua tin hieu ADC THO cua CHINH NO
    // trong 1 khung, dung ket qua do de khuech dai bien do cho
    // khung KE TIEP CUA KENH DO. Nho vay CH1/CH2 co the o 2 che
    // do khac nhau (vd CH1 = AGC, CH2 = chinh tay) hoac 2 muc gain
    // khac nhau ma khong anh huong lan nhau.
    // Chi dung phep nhan luy thua 2 (shift), khong dung phep chia.
    // =========================================================

    reg [9:0] adc_min1, adc_max1;        // CH1: dang do trong khung hien tai
    reg [9:0] scale_min1, scale_max1;    // CH1: chot tu khung truoc

    reg [9:0] adc_min2, adc_max2;        // CH2: dang do trong khung hien tai
    reg [9:0] scale_min2, scale_max2;    // CH2: chot tu khung truoc

    // agc_mode_chX (tu top.v, dieu khien boi num xoay VDIV) thay
    // cho parameter AGC_ENABLE co dinh truoc day.
    wire [9:0] eff_min1   = agc_mode_ch1 ? scale_min1 : 10'd0;
    wire [9:0] eff_max1   = agc_mode_ch1 ? scale_max1 : 10'd1023;
    wire [9:0] agc_range1 = eff_max1 - eff_min1;
    wire [9:0] agc_mid1   = eff_min1 + (agc_range1 >> 1);

    wire [9:0] eff_min2   = agc_mode_ch2 ? scale_min2 : 10'd0;
    wire [9:0] eff_max2   = agc_mode_ch2 ? scale_max2 : 10'd1023;
    wire [9:0] agc_range2 = eff_max2 - eff_min2;
    wire [9:0] agc_mid2   = eff_min2 + (agc_range2 >> 1);

    reg [3:0] gain_shift_auto1, gain_shift_auto2;

    always @(*) begin
        if      (agc_range1 >= 10'd512) gain_shift_auto1 = 4'd0;
        else if (agc_range1 >= 10'd256) gain_shift_auto1 = 4'd1;
        else if (agc_range1 >= 10'd128) gain_shift_auto1 = 4'd2;
        else if (agc_range1 >= 10'd64)  gain_shift_auto1 = 4'd3;
        else if (agc_range1 >= 10'd32)  gain_shift_auto1 = 4'd4;
        else if (agc_range1 >= 10'd16)  gain_shift_auto1 = 4'd5;
        else                            gain_shift_auto1 = 4'd6;  // gioi han gain toi da 64x
    end

    always @(*) begin
        if      (agc_range2 >= 10'd512) gain_shift_auto2 = 4'd0;
        else if (agc_range2 >= 10'd256) gain_shift_auto2 = 4'd1;
        else if (agc_range2 >= 10'd128) gain_shift_auto2 = 4'd2;
        else if (agc_range2 >= 10'd64)  gain_shift_auto2 = 4'd3;
        else if (agc_range2 >= 10'd32)  gain_shift_auto2 = 4'd4;
        else if (agc_range2 >= 10'd16)  gain_shift_auto2 = 4'd5;
        else                            gain_shift_auto2 = 4'd6;  // gioi han gain toi da 64x
    end

    // =========================================================
    // SCALE V/DIV THUC
    //
    // AGC mode: giu cach tinh gain tu code cu.
    // Manual mode: vdiv_index chon V/DIV that:
    //   0 = 100mV/div
    //   1 = 200mV/div
    //   2 = 500mV/div
    //   3 = 1.0V/div
    //   4 = 2.0V/div
    //   5 = 5.0V/div
    //   6 = 10.0V/div
    //
    // ADC 10-bit, Vref = 3.3V, tam man hinh = ADC 512.
    // 24 pixel = 1 division.
    // Dung fixed-point Q16 de khong can phep chia trong FPGA.
    // =========================================================

    reg [31:0] manual_scale_q16_1;   // CH1
    reg [31:0] manual_scale_q16_2;   // CH2

    always @(*) begin
        case (vdiv_index_ch1)
            4'd0: manual_scale_q16_1 = 32'd50738; // 100mV/div
            4'd1: manual_scale_q16_1 = 32'd25369; // 200mV/div
            4'd2: manual_scale_q16_1 = 32'd10148; // 500mV/div
            4'd3: manual_scale_q16_1 = 32'd5074;  // 1.0V/div
            4'd4: manual_scale_q16_1 = 32'd2537;  // 2.0V/div
            4'd5: manual_scale_q16_1 = 32'd1015;  // 5.0V/div
            default: manual_scale_q16_1 = 32'd507; // 10.0V/div
        endcase
    end

    always @(*) begin
        case (vdiv_index_ch2)
            4'd0: manual_scale_q16_2 = 32'd50738; // 100mV/div
            4'd1: manual_scale_q16_2 = 32'd25369; // 200mV/div
            4'd2: manual_scale_q16_2 = 32'd10148; // 500mV/div
            4'd3: manual_scale_q16_2 = 32'd5074;  // 1.0V/div
            4'd4: manual_scale_q16_2 = 32'd2537;  // 2.0V/div
            4'd5: manual_scale_q16_2 = 32'd1015;  // 5.0V/div
            default: manual_scale_q16_2 = 32'd507; // 10.0V/div
        endcase
    end

    // ---- AGC (giu cach tinh cu, nay dung mid/gain RIENG CH1) ----
    wire signed [17:0] a1_centered_agc =
        $signed({8'b0, wave_sample}) - $signed({8'b0, agc_mid1});
    wire signed [17:0] a1_amplified_agc = a1_centered_agc <<< gain_shift_auto1;
    wire signed [17:0] a1_recentered_agc = a1_amplified_agc + 18'sd511;

    wire [9:0] a1_sample_agc =
        (a1_recentered_agc < 18'sd0) ? 10'd0 :
        (a1_recentered_agc > 18'sd1023) ? 10'd1023 :
        a1_recentered_agc[9:0];

    wire [17:0] a1_scaled_agc = a1_sample_agc * 18'd192;
    wire [7:0] a1_y_agc = a1_scaled_agc[17:10];

    // ---- AGC (RIENG CH2) ----
    wire signed [17:0] a2_centered_agc =
        $signed({8'b0, wave_sample2}) - $signed({8'b0, agc_mid2});
    wire signed [17:0] a2_amplified_agc = a2_centered_agc <<< gain_shift_auto2;
    wire signed [17:0] a2_recentered_agc = a2_amplified_agc + 18'sd511;

    wire [9:0] a2_sample_agc =
        (a2_recentered_agc < 18'sd0) ? 10'd0 :
        (a2_recentered_agc > 18'sd1023) ? 10'd1023 :
        a2_recentered_agc[9:0];

    wire [17:0] a2_scaled_agc = a2_sample_agc * 18'd192;
    wire [7:0] a2_y_agc = a2_scaled_agc[17:10];

    // ---- MANUAL V/DIV (RIENG CH1/CH2) ----
    // 1 ADC count -> manual_scale_q16_chX / 65536 pixel.
    wire signed [10:0] adc_delta1 =
        $signed({1'b0, wave_sample}) - 11'sd512;
    wire signed [10:0] adc_delta2 =
        $signed({1'b0, wave_sample2}) - 11'sd512;

    wire signed [43:0] pix_fp1 =
        adc_delta1 * $signed({1'b0, manual_scale_q16_1[15:0]});
    wire signed [43:0] pix_fp2 =
        adc_delta2 * $signed({1'b0, manual_scale_q16_2[15:0]});

    wire signed [31:0] pix1 = pix_fp1 >>> 16;
    wire signed [31:0] pix2 = pix_fp2 >>> 16;

    wire signed [31:0] y1_manual = 32'sd120 - pix1;
    wire signed [31:0] y2_manual = 32'sd120 - pix2;

    wire [7:0] a1_y_manual =
        (y1_manual < 32'sd24) ? 8'd24 :
        (y1_manual > 32'sd215) ? 8'd215 :
        y1_manual[7:0];

    wire [7:0] a2_y_manual =
        (y2_manual < 32'sd24) ? 8'd24 :
        (y2_manual > 32'sd215) ? 8'd215 :
        y2_manual[7:0];

    // a1_y/a2_y la toa do Y cuoi cung ma scope_ui su dung truc tiep.
    wire [7:0] a1_y = agc_mode_ch1 ? (8'd215 - a1_y_agc) : a1_y_manual;
    wire [7:0] a2_y = agc_mode_ch2 ? (8'd215 - a2_y_agc) : a2_y_manual;

    // =========================================================
    // Cong do lech Y vao toa do Y roi chan trong vung song 24..215
    // =========================================================

    function [7:0] apply_off;
        input [7:0]        y;
        input signed [7:0] off;
        reg signed [9:0]   sum;
        begin
            sum = $signed({2'b00, y}) + $signed({{2{off[7]}}, off});
            if (sum < 10'sd24)
                apply_off = 8'd24;
            else if (sum > 10'sd215)
                apply_off = 8'd215;
            else
                apply_off = sum[7:0];
        end
    endfunction

    // =========================================================
    // TRIGGER (THEM MOI) - canh len, kenh trigger = kenh dang duoc
    // num VDIV dieu khien (sel_ch: 0 = CH1, 1 = CH2)
    //
    // Muc trigger tu dong = trung diem (min+max)/2 cua khung truoc
    // cua kenh do. Hysteresis = range/8, toi thieu 2.
    // trig_y = toa do sy cua vach trigger tren man hinh:
    //   - AGC   : muc trigger = tam bien do -> sy = 120
    //   - Manual: tinh giong a1_y_manual, voi muc trigger thay mau
    // =========================================================

    assign cap_active = capture_active;

    wire [9:0]  t_min   = sel_ch ? scale_min2 : scale_min1;
    wire [9:0]  t_max   = sel_ch ? scale_max2 : scale_max1;
    wire [10:0] t_sum   = t_min + t_max;
    wire [9:0]  t_range = t_max - t_min;

    assign trig_level = t_sum[10:1];
    assign trig_hys   = (t_range[9:3] < 7'd2) ? 10'd2 : {3'b000, t_range[9:3]};
    assign trig_valid = (t_range >= 10'd16);

    wire signed [10:0] t_delta = $signed({1'b0, trig_level}) - 11'sd512;
    wire [31:0]        t_scale = sel_ch ? manual_scale_q16_2 : manual_scale_q16_1;
    wire signed [43:0] t_fp    = t_delta * $signed({1'b0, t_scale[15:0]});
    wire signed [31:0] t_pix   = t_fp >>> 16;
    wire signed [31:0] t_ym    = 32'sd120 - t_pix;

    wire [7:0] trig_y_manual =
        (t_ym < 32'sd24)  ? 8'd24  :
        (t_ym > 32'sd215) ? 8'd215 :
        t_ym[7:0];

    wire       t_agc  = sel_ch ? agc_mode_ch2 : agc_mode_ch1;
    wire [7:0] trig_y_base = t_agc ? 8'd120 : trig_y_manual;
    wire [7:0] trig_y      = apply_off(trig_y_base, sel_ch ? off_ch2 : off_ch1);

    // Bao hieu FSM ve man hinh vua ghi xong pixel CUOI CUNG cua
    // khung hinh (hang cuoi = 319, cot cuoi = 239).
    wire frame_done_pulse = (state == ST_PIXEL_LOW) && spi_done &&
                             (x_cnt == 9'd239) && (y_cnt == 9'd319);

    // time_index chot theo khung, de nhan TIME/DIV khong bi doi
    // giua chung luc dang ve (tranh nua trai chu cu, nua phai chu moi)
    reg [3:0] ti_frame;

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            row_ptr        <= 9'd0;
            capture_active <= 1'b1;

            adc_min1   <= 10'd1023;
            adc_max1   <= 10'd0;
            scale_min1 <= 10'd0;
            scale_max1 <= 10'd1023;   // khung dau tien: chua co so lieu -> khong khuech dai

            adc_min2   <= 10'd1023;
            adc_max2   <= 10'd0;
            scale_min2 <= 10'd0;
            scale_max2 <= 10'd1023;

            ti_frame  <= 4'd3;

        end

        else if (frame_done_pulse) begin

            // Man vua ve xong 1 khung -> cho phep quet/nap lai
            // wave_buf tu dau cho khung tiep theo, dong thoi chot
            // lai min/max vua do duoc de lam gain cho khung SAU.
            capture_active <= 1'b1;
            row_ptr        <= 9'd0;

            // Chi chot min/max khi khung vua roi CO nap mau moi
            // (khi STOP khong co mau moi -> giu nguyen gain/trigger cu)
            if (adc_max1 >= adc_min1) begin
                scale_min1 <= adc_min1;
                scale_max1 <= adc_max1;
                scale_min2 <= adc_min2;
                scale_max2 <= adc_max2;
            end

            adc_min1 <= 10'd1023;
            adc_max1 <= 10'd0;
            adc_min2 <= 10'd1023;
            adc_max2 <= 10'd0;

            // Khi STOP giu nhan TIME/DIV cua khung dang dong bang
            if (run)
                ti_frame <= time_index;

        end

        else if (wave_push && capture_active) begin

            if (wave_sample  < adc_min1) adc_min1 <= wave_sample;
            if (wave_sample  > adc_max1) adc_max1 <= wave_sample;
            if (wave_sample2 < adc_min2) adc_min2 <= wave_sample2;
            if (wave_sample2 > adc_max2) adc_max2 <= wave_sample2;

            if (row_ptr == 9'd319) begin

                // Da nap du 320 diem cho 1 lan quet -> dong bang lai,
                // cho man ve xong roi moi nap tiep (frame_done_pulse).
                row_ptr        <= 9'd0;
                capture_active <= 1'b0;

            end
            else begin

                row_ptr <= row_ptr + 1'b1;

            end

        end

    end


    // ---- ghi / doc block RAM ----

    wire buf_we = rst_n && !frame_done_pulse && wave_push && capture_active;

    reg [7:0] q1, q2;

    always @(posedge clk) begin

        if (buf_we) begin
            wave_buf1[row_ptr] <= a1_y;
            wave_buf2[row_ptr] <= a2_y;
        end

        q1 <= wave_buf1[y_cnt];
        q2 <= wave_buf2[y_cnt];

    end


    // =========================================================
    // TINH MAU PIXEL HIEN TAI  -> giao cho scope_ui
    //
    // Can: mau tai cot hien tai (q) va mau cot truoc do (prev_hold)
    // de noi 2 mau bang doan thang. prev_hold chot lai gia tri cu
    // ngay khi y_cnt sang cot moi (q luc do van con du lieu cot cu).
    // =========================================================

    reg [8:0] col_last;
    reg [7:0] prev1_hold, prev2_hold;

    always @(posedge clk) begin

        col_last <= y_cnt;

        if (y_cnt != col_last) begin
            prev1_hold <= q1;
            prev2_hold <= q2;
        end

    end

    // Cong do lech VERTICAL POSITION luc hien thi (dich ngay, ke ca khi STOP)
    wire [7:0] cur1_val  = apply_off(q1, off_ch1);
    wire [7:0] cur2_val  = apply_off(q2, off_ch2);
    wire [7:0] prev1_val = apply_off((y_cnt == 9'd0) ? q1 : prev1_hold, off_ch1);
    wire [7:0] prev2_val = apply_off((y_cnt == 9'd0) ? q2 : prev2_hold, off_ch2);

    wire [15:0] pixel_color;

    scope_ui u_ui (

        .clk         (clk),
        .rst_n       (rst_n),

        .sx          (y_cnt),
        .sy          (x_cnt),

        .time_index  (ti_frame),

        .vdiv_index_ch1 (vdiv_index_ch1),
        .vdiv_index_ch2 (vdiv_index_ch2),
        .agc_mode_ch1   (agc_mode_ch1),
        .agc_mode_ch2   (agc_mode_ch2),
        .sel_ch         (sel_ch),
        .freq1_hz       (freq1_hz),
        .freq2_hz       (freq2_hz),
        .trig_y         (trig_y),
        .vpp_raw        (t_range),
        .run            (run),
        .trig_mode      (trig_mode),
        .trig_slope     (trig_slope),
        .single_done    (single_done),

        .trace1_cur  (cur1_val),
        .trace1_prev (prev1_val),
        .trace2_cur  (cur2_val),
        .trace2_prev (prev2_val),

        .color       (pixel_color)

    );


    // =========================================================
    // MAIN PROCESS - GIU NGUYEN 100% PHAN DIA CHI/KHOI TAO NHU
    // BAN GOC DA CHAY DUNG
    // =========================================================

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            TFT_CS   <= 1'b1;
            TFT_DC   <= 1'b0;
            TFT_RST  <= 1'b0;
            TFT_SCK  <= 1'b0;
            TFT_MOSI <= 1'b0;

            ready <= 1'b0;

            state <= ST_RESET;

            spi_shift   <= 8'h00;
            spi_bit     <= 4'd0;
            spi_div_cnt <= 8'd0;
            spi_busy    <= 1'b0;
            spi_done    <= 1'b0;

            tx_byte    <= 8'h00;
            tx_request <= 1'b0;

            wait_cnt <= 23'd0;

            x_cnt <= 9'd0;
            y_cnt <= 9'd0;

        end

        else begin

            spi_done    <= 1'b0;
            tx_request  <= 1'b0;

            if (tx_request && !spi_busy) begin

                spi_shift   <= tx_byte;
                spi_bit     <= 4'd7;
                spi_div_cnt <= 8'd0;

                spi_busy <= 1'b1;

                TFT_SCK  <= 1'b0;
                TFT_MOSI <= tx_byte[7];

            end

            else if (spi_busy) begin

                if (spi_div_cnt == SPI_DIV - 1) begin

                    spi_div_cnt <= 8'd0;

                    if (TFT_SCK == 1'b0) begin

                        TFT_SCK <= 1'b1;

                    end

                    else begin

                        TFT_SCK <= 1'b0;

                        if (spi_bit == 0) begin

                            spi_busy <= 1'b0;
                            spi_done <= 1'b1;
                            TFT_MOSI <= 1'b0;

                        end

                        else begin

                            spi_bit <= spi_bit - 1'b1;
                            TFT_MOSI <= spi_shift[spi_bit - 1'b1];

                        end

                    end

                end

                else begin

                    spi_div_cnt <= spi_div_cnt + 1'b1;

                end

            end


            case (state)

                ST_RESET: begin

                    TFT_CS  <= 1'b1;
                    TFT_DC  <= 1'b0;
                    TFT_RST <= 1'b0;
                    TFT_SCK  <= 1'b0;
                    TFT_MOSI <= 1'b0;
                    ready <= 1'b0;

                    if (wait_cnt < 23'd5_000_000) begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                    else begin
                        wait_cnt <= 23'd0;
                        TFT_RST <= 1'b1;
                        state <= ST_RESET_WAIT;
                    end

                end

                ST_RESET_WAIT: begin

                    TFT_RST <= 1'b1;
                    TFT_CS <= 1'b1;

                    if (wait_cnt < 23'd5_000_000) begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                    else begin
                        wait_cnt <= 23'd0;
                        TFT_CS <= 1'b0;
                        TFT_DC <= 1'b0;
                        tx_byte <= 8'h01;
                        tx_request <= 1'b1;
                        state <= ST_SWRESET;
                    end

                end

                ST_SWRESET: begin
                    if (spi_done) begin
                        TFT_CS <= 1'b1;
                        wait_cnt <= 23'd0;
                        state <= ST_SWRESET_WAIT;
                    end
                end

                ST_SWRESET_WAIT: begin

                    TFT_CS <= 1'b1;

                    if (wait_cnt < 23'd6_000_000) begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                    else begin
                        wait_cnt <= 23'd0;
                        TFT_CS <= 1'b0;
                        TFT_DC <= 1'b0;
                        tx_byte <= 8'h11;
                        tx_request <= 1'b1;
                        state <= ST_SLEEP;
                    end

                end

                ST_SLEEP: begin
                    if (spi_done) begin
                        TFT_CS <= 1'b1;
                        wait_cnt <= 23'd0;
                        state <= ST_SLEEP_WAIT;
                    end
                end

                ST_SLEEP_WAIT: begin

                    TFT_CS <= 1'b1;

                    if (wait_cnt < 23'd6_000_000) begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                    else begin
                        wait_cnt <= 23'd0;
                        TFT_CS <= 1'b0;
                        TFT_DC <= 1'b0;
                        tx_byte <= 8'h3A;
                        tx_request <= 1'b1;
                        state <= ST_PIXFMT_CMD;
                    end

                end

                ST_PIXFMT_CMD: begin
                    if (spi_done) begin
                        TFT_DC <= 1'b1;
                        tx_byte <= 8'h55;
                        tx_request <= 1'b1;
                        state <= ST_PIXFMT_DATA;
                    end
                end

                ST_PIXFMT_DATA: begin
                    if (spi_done) begin
                        TFT_DC <= 1'b0;
                        TFT_CS <= 1'b1;
                        state <= ST_MADCTL_CMD;
                    end
                end

                ST_MADCTL_CMD: begin
                    TFT_CS <= 1'b0;
                    TFT_DC <= 1'b0;
                    tx_byte <= 8'h36;
                    tx_request <= 1'b1;
                    state <= ST_MADCTL_DATA;
                end

                ST_MADCTL_DATA: begin
                    if (spi_done) begin
                        TFT_DC <= 1'b1;
                        tx_byte <= 8'h48;      // giu nguyen nhu ban goc
                        tx_request <= 1'b1;
                        state <= ST_DISP_CMD;
                    end
                end

                ST_DISP_CMD: begin
                    if (spi_done) begin
                        TFT_DC <= 1'b0;
                        tx_byte <= 8'h29;
                        tx_request <= 1'b1;
                        state <= ST_DISP_WAIT;
                    end
                end

                ST_DISP_WAIT: begin
                    if (spi_done) begin
                        TFT_CS <= 1'b1;
                        TFT_DC <= 1'b0;
                        wait_cnt <= 23'd0;
                        state <= ST_CASET_CMD;
                    end
                end

                ST_CASET_CMD: begin
                    TFT_CS <= 1'b0;
                    TFT_DC <= 1'b0;
                    tx_byte <= 8'h2A;
                    tx_request <= 1'b1;
                    state <= ST_CASET_1;
                end

                ST_CASET_1: begin
                    if (spi_done) begin
                        TFT_DC <= 1'b1;
                        tx_byte <= 8'h00;
                        tx_request <= 1'b1;
                        state <= ST_CASET_2;
                    end
                end

                ST_CASET_2: begin
                    if (spi_done) begin
                        tx_byte <= 8'h00;
                        tx_request <= 1'b1;
                        state <= ST_CASET_3;
                    end
                end

                ST_CASET_3: begin
                    if (spi_done) begin
                        tx_byte <= 8'h00;
                        tx_request <= 1'b1;
                        state <= ST_CASET_4;
                    end
                end

                ST_CASET_4: begin
                    if (spi_done) begin
                        tx_byte <= 8'hEF;      // 239 - giu nguyen ban goc
                        tx_request <= 1'b1;
                        state <= ST_PASET_CMD;
                    end
                end

                ST_PASET_CMD: begin
                    if (spi_done) begin
                        TFT_DC <= 1'b0;
                        tx_byte <= 8'h2B;
                        tx_request <= 1'b1;
                        state <= ST_PASET_1;
                    end
                end

                ST_PASET_1: begin
                    if (spi_done) begin
                        TFT_DC <= 1'b1;
                        tx_byte <= 8'h00;
                        tx_request <= 1'b1;
                        state <= ST_PASET_2;
                    end
                end

                ST_PASET_2: begin
                    if (spi_done) begin
                        tx_byte <= 8'h00;
                        tx_request <= 1'b1;
                        state <= ST_PASET_3;
                    end
                end

                ST_PASET_3: begin
                    if (spi_done) begin
                        tx_byte <= 8'h01;
                        tx_request <= 1'b1;
                        state <= ST_PASET_4;
                    end
                end

                ST_PASET_4: begin
                    if (spi_done) begin
                        tx_byte <= 8'h3F;      // 319 - giu nguyen ban goc
                        tx_request <= 1'b1;
                        state <= ST_RAMWR_CMD;
                    end
                end

                ST_RAMWR_CMD: begin
                    if (spi_done) begin
                        TFT_DC <= 1'b0;
                        tx_byte <= 8'h2C;
                        tx_request <= 1'b1;
                        x_cnt <= 9'd0;
                        y_cnt <= 9'd0;
                        state <= ST_PIXEL_HIGH;
                    end
                end

                ST_PIXEL_HIGH: begin
                    if (spi_done) begin
                        TFT_DC <= 1'b1;
                        tx_byte <= pixel_color[15:8];
                        tx_request <= 1'b1;
                        state <= ST_PIXEL_LOW;
                    end
                end

                ST_PIXEL_LOW: begin
                    if (spi_done) begin

                        tx_byte <= pixel_color[7:0];
                        tx_request <= 1'b1;

                        if (x_cnt == 9'd239) begin

                            x_cnt <= 9'd0;

                            if (y_cnt == 9'd319) begin

                                y_cnt <= 9'd0;
                                ready <= 1'b1;
                                state <= ST_RAMWR_CMD;

                            end
                            else begin
                                y_cnt <= y_cnt + 1'b1;
                                state <= ST_PIXEL_HIGH;
                            end

                        end
                        else begin
                            x_cnt <= x_cnt + 1'b1;
                            state <= ST_PIXEL_HIGH;
                        end

                    end
                end

                default: begin

                    TFT_CS   <= 1'b1;
                    TFT_DC   <= 1'b0;
                    TFT_RST  <= 1'b0;
                    TFT_SCK  <= 1'b0;
                    TFT_MOSI <= 1'b0;
                    ready <= 1'b0;
                    wait_cnt <= 23'd0;
                    state <= ST_RESET;

                end

            endcase

        end

    end

endmodule