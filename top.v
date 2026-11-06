module top ( 
    input  wire       CLOCK_50, 
    input  wire       RESET_N, 
 
    // ========================================================= 
    // EC11 
    // ========================================================= 
 
    input  wire       EC11_A, 
    input  wire       EC11_B, 
    input  wire       EC11_SW, 

    // =========================================================
    // EC11 thu 2 - V/DIV (do nhay doc)
    // =========================================================

    input  wire       VDIV_A,
    input  wire       VDIV_B,
    input  wire       VDIV_SW,

    // =========================================================
    // EC11 thu 3 - VERTICAL POSITION (dich song len/xuong)
    // =========================================================

    input  wire       POS_A,
    input  wire       POS_B,
    input  wire       POS_SW,
 
    // ========================================================= 
    // MCP3008 
    // ========================================================= 
 
    output wire       MCP_CS, 
    output wire       MCP_DIN, 
    input  wire       MCP_DOUT, 
    output wire       MCP_CLK, 
 
    // ========================================================= 
    // LED 
    // ========================================================= 
 
    output wire [4:0] LED, 
 
    // ========================================================= 
    // UART 
    // ========================================================= 
 
    output wire       UART_TX, 
 
    // ========================================================= 
    // TFT ILI9341 
    // ========================================================= 
 
    output wire       TFT_CS, 
    output wire       TFT_DC, 
    output wire       TFT_RST, 
    output wire       TFT_SCK, 
    output wire       TFT_MOSI 
); 


    // =========================================================
    // RESET SYNC - assert bat dong bo, nha (release) dong bo
    // voi CLOCK_50 de tranh metastability khi nha nut RESET_N.
    // Cac khoi ben duoi dung rst_n thay cho RESET_N.
    // =========================================================

    reg  rst_ff1, rst_ff2;

    always @(posedge CLOCK_50 or negedge RESET_N) begin

        if (!RESET_N) begin

            rst_ff1 <= 1'b0;
            rst_ff2 <= 1'b0;

        end

        else begin

            rst_ff1 <= 1'b1;
            rst_ff2 <= rst_ff1;

        end

    end

    wire rst_n = rst_ff2;


    // =========================================================
    // MCP3008
    // =========================================================

    wire [9:0] adc_data;
    wire       adc_valid;
    wire       adc_ch;         // 0 = mau CH1 (MCP3008 CH0), 1 = mau CH2 (MCP3008 CH1)


    mcp3008_2ch u_adc (

        .clk       (CLOCK_50),
        .rst_n     (rst_n),

        .adc_cs    (MCP_CS),
        .adc_clk   (MCP_CLK),
        .adc_din   (MCP_DIN),
        .adc_dout  (MCP_DOUT),

        .adc_data  (adc_data),
        .adc_valid (adc_valid),
        .adc_ch    (adc_ch)

    );


    // ADC doc luan phien CH1, CH2, CH1, CH2...
    // Giu mau CH1, khi mau CH2 den thi co du 1 cap (CH1, CH2).

    reg [9:0] ch1_hold;

    always @(posedge CLOCK_50) begin

        if (!rst_n)

            ch1_hold <= 10'd0;

        else if (adc_valid && !adc_ch)

            ch1_hold <= adc_data;

    end

    wire pair_valid = adc_valid && adc_ch;


    // =========================================================
    // EC11
    // =========================================================

    wire              ec11_step;     // 1 xung / 1 nac xoay
    wire              ec11_dir;      // 1 = xoay thuan, 0 = xoay nguoc


    rotary_encoder u_ec11 (

        .clk          (CLOCK_50),
        .rst_n        (rst_n),

        .enc_a        (EC11_A),
        .enc_b        (EC11_B),

        .step         (ec11_step),
        .dir          (ec11_dir)

    );


    // =========================================================
    // POS_SW (nut cua num VERTICAL POSITION) - tu khu doi + phan
    // biet nhan nhanh / giu lau (giong nut VDIV):
    //   short press -> RUN / STOP (dong bang khung hinh)
    //   long  press -> dua song cua kenh dang chon ve vi tri goc
    // Gia dinh POS_SW active-low (nhan -> keo xuong GND).
    // =========================================================

    reg        ec_sw_ff1, ec_sw_ff2;
    reg [19:0] ec_db_cnt;
    reg        ec_sw_stable;
    reg        ec_sw_stable_d;
    reg [25:0] ec_press_cnt;

    localparam [19:0] EC_DB_LIMIT   = 20'd499_999;     // ~10ms @ 50MHz
    localparam [25:0] EC_HOLD_LIMIT = 26'd25_000_000;  // ~500ms @ 50MHz

    reg ec_long_pulse;

    reg run;   // 1 = RUN, 0 = STOP (dong bang)


    always @(posedge CLOCK_50) begin

        if (!rst_n) begin

            ec_sw_ff1      <= 1'b1;
            ec_sw_ff2      <= 1'b1;
            ec_db_cnt      <= 20'd0;
            ec_sw_stable   <= 1'b1;
            ec_sw_stable_d <= 1'b1;
            ec_press_cnt   <= 26'd0;
            ec_long_pulse  <= 1'b0;

            run            <= 1'b1;

        end

        else begin

            ec_sw_ff1 <= POS_SW;
            ec_sw_ff2 <= ec_sw_ff1;

            ec_long_pulse  <= 1'b0;

            // Khu doi
            if (ec_sw_ff2 == ec_sw_stable) begin

                ec_db_cnt <= 20'd0;

            end

            else if (ec_db_cnt == EC_DB_LIMIT) begin

                ec_sw_stable <= ec_sw_ff2;

            end

            else begin

                ec_db_cnt <= ec_db_cnt + 1'b1;

            end

            ec_sw_stable_d <= ec_sw_stable;

            if (!ec_sw_stable) begin

                // Dang giu nut: bao long-press dung 1 lan khi vuot nguong
                if (ec_press_cnt < EC_HOLD_LIMIT) begin

                    ec_press_cnt <= ec_press_cnt + 1'b1;

                    if (ec_press_cnt == EC_HOLD_LIMIT - 1'b1)

                        ec_long_pulse <= 1'b1;

                end

            end

            else begin

                // Vua tha nut: chua kip thanh long-press -> short-press
                if (!ec_sw_stable_d && (ec_press_cnt < EC_HOLD_LIMIT)) begin

                    run <= ~run;       // nhan nhanh -> RUN/STOP

                end

                ec_press_cnt <= 26'd0;

            end

        end

    end


    // =========================================================
    // TRIGGER MODE / SLOPE - nut EC11_SW (nut cua num TIME/DIV)
    //
    //   Nhan nhanh (short) : doi che do  AUTO -> NORMAL -> SINGLE -> AUTO
    //   Giu lau   (long)   : doi canh trigger  LEN <-> XUONG
    //
    //   AUTO   : co canh thi trigger, 100 ms khong co canh -> tu chay
    //   NORMAL : chi ve khi co canh trigger (khong co thi giu khung cu)
    //   SINGLE : cho 1 canh trigger, thu 1 khung roi dung. Muon thu lai:
    //            nhan POS_SW 2 lan (STOP -> RUN) hoac doi che do.
    //
    //   LED[0]=AUTO  LED[1]=NORMAL  LED[2]=SINGLE
    //   LED[3]=canh XUONG (tat = canh LEN)
    //   LED[4]=SINGLE da thu xong (dang giu khung)
    //
    // Gia dinh EC11_SW active-low (nhan -> keo xuong GND).
    // LUU Y: chuc nang cu "nhan EC11_SW -> ve 1.6ms/div" da duoc thay.
    // =========================================================

    localparam [1:0] TM_AUTO   = 2'd0,
                     TM_NORMAL = 2'd1,
                     TM_SINGLE = 2'd2;

    reg [1:0]  trig_mode;
    reg        trig_slope;     // 0 = canh len, 1 = canh xuong
    reg        single_done;    // SINGLE: da thu xong 1 khung, dang giu
    reg        run_d;

    reg        tm_sw_ff1, tm_sw_ff2;
    reg [19:0] tm_db_cnt;
    reg        tm_sw_stable;
    reg        tm_sw_stable_d;
    reg [25:0] tm_press_cnt;
    reg        tm_short_pulse;

    localparam [19:0] TM_DB_LIMIT   = 20'd499_999;     // ~10ms @ 50MHz
    localparam [25:0] TM_HOLD_LIMIT = 26'd25_000_000;  // ~500ms @ 50MHz


    always @(posedge CLOCK_50) begin

        if (!rst_n) begin

            tm_sw_ff1      <= 1'b1;
            tm_sw_ff2      <= 1'b1;
            tm_db_cnt      <= 20'd0;
            tm_sw_stable   <= 1'b1;
            tm_sw_stable_d <= 1'b1;
            tm_press_cnt   <= 26'd0;
            tm_short_pulse <= 1'b0;

            trig_mode      <= TM_AUTO;
            trig_slope     <= 1'b0;

        end

        else begin

            tm_sw_ff1 <= EC11_SW;
            tm_sw_ff2 <= tm_sw_ff1;

            tm_short_pulse <= 1'b0;

            // Khu doi
            if (tm_sw_ff2 == tm_sw_stable)
                tm_db_cnt <= 20'd0;
            else if (tm_db_cnt == TM_DB_LIMIT)
                tm_sw_stable <= tm_sw_ff2;
            else
                tm_db_cnt <= tm_db_cnt + 1'b1;

            tm_sw_stable_d <= tm_sw_stable;

            if (!tm_sw_stable) begin

                // Dang giu: vua vuot nguong -> doi canh trigger (1 lan)
                if (tm_press_cnt < TM_HOLD_LIMIT) begin

                    tm_press_cnt <= tm_press_cnt + 1'b1;

                    if (tm_press_cnt == TM_HOLD_LIMIT - 1'b1)
                        trig_slope <= ~trig_slope;

                end

            end

            else begin

                // Vua tha, chua kip thanh long-press -> doi che do
                if (!tm_sw_stable_d && (tm_press_cnt < TM_HOLD_LIMIT)) begin

                    tm_short_pulse <= 1'b1;

                    if (trig_mode == TM_SINGLE)
                        trig_mode <= TM_AUTO;
                    else
                        trig_mode <= trig_mode + 1'b1;

                end

                tm_press_cnt <= 26'd0;

            end

        end

    end


    assign LED = {single_done,
                  trig_slope,
                  (trig_mode == TM_SINGLE),
                  (trig_mode == TM_NORMAL),
                  (trig_mode == TM_AUTO)};


    // =========================================================
    // EC11 thu 2 - V/DIV
    // =========================================================

    wire              vdiv_step;
    wire              vdiv_dir;


    rotary_encoder u_vdiv (

        .clk          (CLOCK_50),
        .rst_n        (rst_n),

        .enc_a        (VDIV_A),
        .enc_b        (VDIV_B),

        .step         (vdiv_step),
        .dir          (vdiv_dir)

    );


    // =========================================================
    // VDIV BUTTON - tu khu doi (debounce) + phan biet nhan nhanh
    // (short press) / giu lau (long press) tren chinh VDIV_SW,
    // KHONG dung button_pulse cua module rotary_encoder nua (de
    // tu do lam chu thoi gian giu nut).
    //
    //   short press -> DOI KENH dang duoc num VDIV dieu khien
    //                  (CH1 <-> CH2)
    //   long  press -> TRA kenh DANG CHON ve AGC tu dong
    //
    // Gia dinh VDIV_SW la nut nhan tich cuc muc THAP (active-low,
    // giong da so module EC11: nhan -> keo xuong GND). Neu board
    // cua ban dung muc cao khi nhan, doi 2 dieu kien
    // "!vdiv_sw_stable" / "vdiv_sw_stable" o duoi cho phu hop.
    // =========================================================

    reg        vdiv_sw_ff1, vdiv_sw_ff2;
    reg [19:0] vdiv_db_cnt;
    reg        vdiv_sw_stable;      // da khu doi: 0 = dang nhan
    reg        vdiv_sw_stable_d;
    reg [25:0] vdiv_press_cnt;

    localparam [19:0] VDIV_DB_LIMIT   = 20'd499_999;    // ~10ms @ 50MHz
    localparam [25:0] VDIV_HOLD_LIMIT = 26'd25_000_000; // ~500ms @ 50MHz

    reg vdiv_short_pulse, vdiv_long_pulse;


    always @(posedge CLOCK_50) begin

        if (!rst_n) begin

            vdiv_sw_ff1      <= 1'b1;
            vdiv_sw_ff2      <= 1'b1;
            vdiv_db_cnt      <= 20'd0;
            vdiv_sw_stable   <= 1'b1;
            vdiv_sw_stable_d <= 1'b1;
            vdiv_press_cnt   <= 26'd0;
            vdiv_short_pulse <= 1'b0;
            vdiv_long_pulse  <= 1'b0;

        end

        else begin

            vdiv_sw_ff1 <= VDIV_SW;
            vdiv_sw_ff2 <= vdiv_sw_ff1;

            vdiv_short_pulse <= 1'b0;
            vdiv_long_pulse  <= 1'b0;

            // Khu doi: chi chap nhan trang thai moi khi no giu
            // nguyen du lau (VDIV_DB_LIMIT chu ky).
            if (vdiv_sw_ff2 == vdiv_sw_stable) begin

                vdiv_db_cnt <= 20'd0;

            end

            else if (vdiv_db_cnt == VDIV_DB_LIMIT) begin

                vdiv_sw_stable <= vdiv_sw_ff2;

            end

            else begin

                vdiv_db_cnt <= vdiv_db_cnt + 1'b1;

            end

            vdiv_sw_stable_d <= vdiv_sw_stable;

            if (!vdiv_sw_stable) begin

                // Dang giu nut: dem thoi gian, bao long-press dung
                // 1 lan khi vua vuot nguong (van con dang giu).
                if (vdiv_press_cnt < VDIV_HOLD_LIMIT) begin

                    vdiv_press_cnt <= vdiv_press_cnt + 1'b1;

                    if (vdiv_press_cnt == VDIV_HOLD_LIMIT - 1'b1)

                        vdiv_long_pulse <= 1'b1;

                end

            end

            else begin

                // Vua tha nut: neu chua kip thanh long-press thi
                // tinh la short-press (doi kenh dang chinh).
                if (!vdiv_sw_stable_d && (vdiv_press_cnt < VDIV_HOLD_LIMIT))

                    vdiv_short_pulse <= 1'b1;

                vdiv_press_cnt <= 26'd0;

            end

        end

    end


    // =========================================================
    // V/DIV CONTROL - RIENG CHO TUNG KENH (CH1/CH2)
    //
    // Moi kenh co V/DIV (agc_mode/vdiv_index) rieng. vdiv_sel_ch
    // cho biet num VDIV dang dieu khien kenh nao (0 = CH1,
    // 1 = CH2). Mac dinh ca 2 kenh chay AGC tu dong.
    //
    //   Xoay num VDIV      -> doi 1 muc gain (x1..x64) cua KENH
    //                         DANG CHON, dong thoi ep kenh do
    //                         sang che do chinh tay.
    //   Nhan nhanh (short) -> DOI kenh dang chon (CH1 <-> CH2).
    //   Giu lau  (long)    -> tra kenh DANG CHON ve AGC tu dong.
    // =========================================================

    reg [3:0] vdiv_index_ch1;      // 0..6, dung khi agc_mode_ch1 = 0
    reg [3:0] vdiv_index_ch2;      // 0..6, dung khi agc_mode_ch2 = 0
    reg       agc_mode_ch1;        // 1 = AGC tu dong, 0 = chinh tay - CH1
    reg       agc_mode_ch2;        // 1 = AGC tu dong, 0 = chinh tay - CH2
    reg       vdiv_sel_ch;         // 0 = num VDIV dang chinh CH1, 1 = dang chinh CH2


    always @(posedge CLOCK_50) begin

        if (!rst_n) begin

            vdiv_index_ch1     <= 4'd0;
            vdiv_index_ch2     <= 4'd0;
            agc_mode_ch1       <= 1'b1;
            agc_mode_ch2       <= 1'b1;
            vdiv_sel_ch        <= 1'b0;

        end

        else begin

            if (vdiv_long_pulse) begin

                // Giu num VDIV du lau -> tra kenh DANG CHON ve AGC
                if (vdiv_sel_ch)
                    agc_mode_ch2 <= 1'b1;
                else
                    agc_mode_ch1 <= 1'b1;

            end

            else if (vdiv_short_pulse) begin

                // Nhan nha num VDIV -> doi kenh dang duoc dieu khien
                vdiv_sel_ch <= ~vdiv_sel_ch;

            end

            else if (vdiv_step) begin

                // Xoay num VDIV -> doi gain cua KENH DANG CHON,
                // chuyen kenh do sang che do chinh tay
                if (vdiv_sel_ch) begin

                    agc_mode_ch2 <= 1'b0;

                    if (vdiv_dir) begin

                        if (vdiv_index_ch2 < 4'd6)
                            vdiv_index_ch2 <= vdiv_index_ch2 + 1'b1;

                    end

                    else begin

                        if (vdiv_index_ch2 > 4'd0)
                            vdiv_index_ch2 <= vdiv_index_ch2 - 1'b1;

                    end

                end

                else begin

                    agc_mode_ch1 <= 1'b0;

                    if (vdiv_dir) begin

                        if (vdiv_index_ch1 < 4'd6)
                            vdiv_index_ch1 <= vdiv_index_ch1 + 1'b1;

                    end

                    else begin

                        if (vdiv_index_ch1 > 4'd0)
                            vdiv_index_ch1 <= vdiv_index_ch1 - 1'b1;

                    end

                end

            end

        end

    end


    // =========================================================
    // EC11 thu 3 - VERTICAL POSITION (dich song len/xuong)
    //
    // Moi kenh co 1 do lech Y (pixel) rieng. Num nay dieu khien
    // KENH DANG CHON boi num VDIV (vdiv_sel_ch).
    //
    //   Xoay num     -> dich song cua kenh dang chon (moi nac = 4 px)
    //   Nhan nhanh   -> RUN / STOP (xu ly o khoi POS_SW ben tren)
    //   Giu lau      -> dua song cua kenh dang chon ve vi tri goc (0)
    //
    // Do lech duoc cong luc HIEN THI (trong tft_ili9341) nen song
    // dich ngay lap tuc, ke ca khi dang STOP.
    // Neu xoay thuan ma song di xuong, doi dau trong khoi duoi.
    // =========================================================

    wire              pos_step;
    wire              pos_dir;


    rotary_encoder u_pos (

        .clk          (CLOCK_50),
        .rst_n        (rst_n),

        .enc_a        (POS_A),
        .enc_b        (POS_B),

        .step         (pos_step),
        .dir          (pos_dir)

    );


    reg signed [7:0] pos_off_ch1;      // do lech Y cua CH1 (pixel, + = xuong)
    reg signed [7:0] pos_off_ch2;      // do lech Y cua CH2

    localparam signed [7:0] POS_STEP =  8'sd4;
    localparam signed [7:0] POS_MAX  =  8'sd80;
    localparam signed [7:0] POS_MIN  = -8'sd80;


    always @(posedge CLOCK_50) begin

        if (!rst_n) begin

            pos_off_ch1       <= 8'sd0;
            pos_off_ch2       <= 8'sd0;

        end

        else begin

            if (ec_long_pulse) begin

                // Giu num -> ve vi tri goc cua kenh dang chon
                if (vdiv_sel_ch)
                    pos_off_ch2 <= 8'sd0;
                else
                    pos_off_ch1 <= 8'sd0;

            end

            else if (pos_step) begin

                if (pos_dir) begin

                    // Xoay thuan -> song di LEN (Y giam)
                    if (vdiv_sel_ch) begin
                        if (pos_off_ch2 > POS_MIN)
                            pos_off_ch2 <= pos_off_ch2 - POS_STEP;
                    end
                    else begin
                        if (pos_off_ch1 > POS_MIN)
                            pos_off_ch1 <= pos_off_ch1 - POS_STEP;
                    end

                end

                else begin

                    // Xoay nguoc -> song di XUONG (Y tang)
                    if (vdiv_sel_ch) begin
                        if (pos_off_ch2 < POS_MAX)
                            pos_off_ch2 <= pos_off_ch2 + POS_STEP;
                    end
                    else begin
                        if (pos_off_ch1 < POS_MAX)
                            pos_off_ch1 <= pos_off_ch1 + POS_STEP;
                    end

                end

            end

        end

    end


    // =========================================================
    // TIME / DIV
    //
    // time_index gio gioi han 0..5 (6 muc):
    //
    //   0 : 200us  - ZOOM IN x8 (push_repeat = 8)
    //   1 : 400us  - ZOOM IN x4 (push_repeat = 4)
    //   2 : 800us  - ZOOM IN x2 (push_repeat = 2)
    //   3 : 1.6ms  - 1:1 (mac dinh)
    //   4 : 3.2ms  - ZOOM OUT, gop 2 mau/cot
    //   5 : 6.4ms  - ZOOM OUT, gop 4 mau/cot
    // =========================================================

    reg [3:0] time_index;

    reg ctrl_pending;

    reg [3:0] ctrl_index;


    // =========================================================
    // UART SIGNALS
    // =========================================================

    reg [5:0] uart_state;

    reg [7:0] uart_data;
    reg       uart_valid;

    wire uart_busy;


    // =========================================================
    // TIME/DIV CONTROL
    // =========================================================

    always @(posedge CLOCK_50) begin

        if (!rst_n) begin

            time_index         <= 4'd3;   // 1.6ms/div (1:1)

            ctrl_pending       <= 1'b1;

            ctrl_index         <= 4'd3;

        end

        else begin

            if (ec11_step) begin

                if (ec11_dir) begin

                    if (time_index < 4'd5) begin

                        time_index   <= time_index + 1'b1;

                        ctrl_index   <= time_index + 1'b1;

                        ctrl_pending <= 1'b1;

                    end

                end

                else begin

                    if (time_index > 4'd0) begin

                        time_index   <= time_index - 1'b1;

                        ctrl_index   <= time_index - 1'b1;

                        ctrl_pending <= 1'b1;

                    end

                end

            end


            if (uart_state == 6'd34 && !uart_busy)

                ctrl_pending <= 1'b0;

        end

    end


    // =========================================================
    // WAVEFORM PUSH (time/div theo time_index, dùng cho TFT)
    //
    // Cứ mỗi 'samples_per_col' mẫu ADC hợp lệ thì đẩy 1 điểm
    // sang bộ đệm sóng trong tft_ili9341 (1 điểm = 1 cột).
    // 'push_repeat' > 1 (cac muc zoom-in 0..2): mau vua lay duoc
    // se duoc day lap lai push_repeat lan tren push_repeat cot
    // lien tiep thay vi cho da so mau moi -> thu hep khoang thoi
    // gian thuc hien tren toan bo 320 cot -> zoom sau hon vao
    // truc thoi gian ma khong can gop/mat mau ADC.
    //
    // TRIGGER (THEM MOI) - canh len, kenh = kenh dang chon boi num VDIV:
    //   - muc trigger tu dong = (min+max)/2 cua khung truoc (lay tu tft)
    //   - hysteresis = range/8 (toi thieu 2) de chong nhieu
    //   - AUTO: 100 ms khong co canh -> tu chay de man hinh khong bi dung
    //   - mau tai thoi diem trigger la COT DAU TIEN (mep trai man hinh)
    // =========================================================

    reg  [15:0] samples_per_col;
    reg  [3:0]  push_repeat;
    reg  [15:0] col_div_cnt;
    reg  [3:0]  repeat_left;
    reg         wave_push;
    reg  [9:0]  wave_sample;    // CH1
    reg  [9:0]  wave_sample2;   // CH2

    // ---- TRIGGER ----
    wire        cap_active;     // tu tft: 1 = dang cho trigger / dang nap 320 cot
    wire [9:0]  trig_level;     // muc trigger (don vi ADC)
    wire [9:0]  trig_hys;       // hysteresis
    wire        trig_valid;     // 1 = bien do du lon de trigger

    reg         acq_run;        // 1 = da trigger, dang thu 320 cot
    reg         trig_below;     // da xuong duoi (level - hys), san sang bat canh len
    reg         trig_above;     // da len tren (level + hys), san sang bat canh xuong
    reg         trig_by_edge;   // khung dang thu duoc trigger boi canh that (khong phai auto)
    reg  [23:0] trig_wait;      // dem thoi gian cho trigger

    localparam [23:0] TRIG_AUTO_TIMEOUT = 24'd5_000_000;   // 100 ms @ 50 MHz

    // Kenh trigger = kenh dang duoc num VDIV dieu khien (vdiv_sel_ch)
    // Tai pair_valid: CH1 = ch1_hold, CH2 = adc_data
    wire [9:0] trig_smp  = vdiv_sel_ch ? adc_data : ch1_hold;

    wire [9:0]  trig_lo   = trig_level - trig_hys;
    wire [10:0] trig_hi   = {1'b0, trig_level} + {1'b0, trig_hys};

    // STOP hoac SINGLE da thu xong -> khong trigger moi
    wire       armed     = cap_active && !acq_run && run && !single_done;

    wire       edge_rise = trig_below && (trig_smp >= trig_level);
    wire       edge_fall = trig_above && (trig_smp <= trig_level);
    wire       trig_edge = trig_valid && (trig_slope ? edge_fall : edge_rise);

    // Tu chay sau 100 ms: luon o AUTO; o NORMAL/SINGLE chi khi chua co
    // bien do hop le (khung truoc phang -> khong tinh duoc muc trigger,
    // neu khong chay thi se ket vi khung khong bao gio duoc cap nhat).
    wire       auto_fire = (trig_wait >= TRIG_AUTO_TIMEOUT) &&
                           ((trig_mode == TM_AUTO) || !trig_valid);

    wire       trig_hit  = armed && pair_valid && (trig_edge || auto_fire);


    always @(*) begin

        case (time_index)

            // ZOOM IN: khong gop mau (samples_per_col=1), chi lap
            // lai tren nhieu cot de "keo dai" 1 mau ra nhieu pixel
            // thoi gian, giam so chu ky song hien tren man hinh.
            4'd0: begin samples_per_col = 16'd1; push_repeat = 4'd8; end
            4'd1: begin samples_per_col = 16'd1; push_repeat = 4'd4; end
            4'd2: begin samples_per_col = 16'd1; push_repeat = 4'd2; end

            // 1:1 - moi mau ADC hop le = 1 cot, khong lap khong gop
            4'd3: begin samples_per_col = 16'd1; push_repeat = 4'd1; end

            // ZOOM OUT: gop nhieu mau ADC lai thanh 1 cot (decimate),
            // giong bang goc, chi doi ten muc (dich +1 so voi ban
            // goc de nhuong cho 3 muc zoom-in o tren).
            4'd4: begin samples_per_col = 16'd2;  push_repeat = 4'd1; end
            4'd5: begin samples_per_col = 16'd4;  push_repeat = 4'd1; end

            default: begin samples_per_col = 16'd16; push_repeat = 4'd1; end

        endcase

    end


    always @(posedge CLOCK_50) begin

        if (!rst_n) begin

            col_div_cnt <= 16'd0;
            repeat_left <= 4'd0;

            wave_push   <= 1'b0;
            wave_sample <= 10'd0;
            wave_sample2 <= 10'd0;

            acq_run     <= 1'b0;
            trig_below  <= 1'b0;
            trig_above  <= 1'b0;
            trig_by_edge <= 1'b0;
            single_done <= 1'b0;
            run_d       <= 1'b1;
            trig_wait   <= 24'd0;

        end

        else begin

            wave_push <= 1'b0;

            // ---- theo doi trang thai "da xuong duoi nguong" ----
            if (pair_valid) begin

                if (trig_smp <= trig_lo)

                    trig_below <= 1'b1;

                else if (trig_smp >= trig_level)

                    trig_below <= 1'b0;

            end

            // ---- theo doi trang thai "da len tren nguong" (canh xuong) ----
            if (pair_valid) begin

                if ({1'b0, trig_smp} >= trig_hi)

                    trig_above <= 1'b1;

                else if (trig_smp <= trig_level)

                    trig_above <= 1'b0;

            end

            // ---- SINGLE: nho da thu xong 1 khung, cho re-arm ----
            //   re-arm: doi che do, hoac STOP -> RUN
            run_d <= run;

            if (tm_short_pulse || (run && !run_d))

                single_done <= 1'b0;

            else if (trig_mode == TM_SINGLE && acq_run && !cap_active && trig_by_edge)

                single_done <= 1'b1;

            // ---- dem thoi gian cho trigger (auto-trigger) ----
            if (armed) begin

                if (trig_wait < TRIG_AUTO_TIMEOUT)

                    trig_wait <= trig_wait + 1'b1;

            end

            else begin

                trig_wait <= 24'd0;

            end

            // ---- thu mau ----
            if (!cap_active) begin

                // Da nap du 320 cot -> cho man ve xong khung
                acq_run     <= 1'b0;
                repeat_left <= 4'd0;
                col_div_cnt <= 16'd0;

            end

            else if (trig_hit) begin

                // Trigger: day NGAY mau nay lam cot dau tien
                acq_run      <= 1'b1;
                trig_by_edge <= trig_edge;

                wave_push    <= 1'b1;
                wave_sample  <= ch1_hold;
                wave_sample2 <= adc_data;

                col_div_cnt  <= 16'd0;
                repeat_left  <= push_repeat - 1'b1;

            end

            else if (acq_run) begin

                if (repeat_left != 4'd0) begin

                    // Dang phat lai (zoom-in) mau vua lay duoc tren cac
                    // cot tiep theo, khong doi wave_sample.
                    wave_push   <= 1'b1;
                    repeat_left <= repeat_left - 1'b1;

                end

                else if (pair_valid) begin

                    if (col_div_cnt >= samples_per_col - 1'b1) begin

                        col_div_cnt <= 16'd0;

                        wave_push   <= 1'b1;
                        wave_sample  <= ch1_hold;
                        wave_sample2 <= adc_data;

                        repeat_left <= push_repeat - 1'b1;

                    end

                    else begin

                        col_div_cnt <= col_div_cnt + 1'b1;

                    end

                end

            end

        end

    end


    // =========================================================
    // FREQUENCY METER - CH1 + CH2
    // MCP3008 2-channel: mỗi kênh = 20 kS/s
    // =========================================================
    wire [31:0] freq1_hz;
    wire [31:0] freq2_hz;
    wire        freq1_valid;
    wire        freq2_valid;

    freq_meter #(
        .SAMPLE_RATE(20000)
    ) u_freq_meter (
        .clk        (CLOCK_50),
        .rst_n      (rst_n),
        .adc_data   (adc_data),
        .adc_valid  (adc_valid),
        .adc_ch     (adc_ch),
        .freq1_hz   (freq1_hz),
        .freq2_hz   (freq2_hz),
        .freq1_valid(freq1_valid),
        .freq2_valid(freq2_valid)
    );

    // =========================================================
    // TFT ILI9341
    //
    // Driver mới:
    // - Vẽ sóng theo bộ đệm 240 cột bên trong module
    // - Nhận điểm mới qua wave_push / wave_sample
    // - Tự lặp lại RAMWR để refresh khung hình liên tục
    // - Nhận time_index để hiển thị TIME/DIV lên màn hình
    // =========================================================

    tft_ili9341 u_tft (

        .clk      (CLOCK_50),
        .rst_n    (rst_n),

        .TFT_CS   (TFT_CS),
        .TFT_DC   (TFT_DC),
        .TFT_RST  (TFT_RST),
        .TFT_SCK  (TFT_SCK),
        .TFT_MOSI (TFT_MOSI),

        .wave_push   (wave_push),
        .wave_sample  (wave_sample),
        .wave_sample2 (wave_sample2),

        .time_index  (time_index),

        .vdiv_index_ch1 (vdiv_index_ch1),
        .vdiv_index_ch2 (vdiv_index_ch2),
        .agc_mode_ch1   (agc_mode_ch1),
        .agc_mode_ch2   (agc_mode_ch2),
        .sel_ch         (vdiv_sel_ch),
        .freq1_hz       (freq1_hz),
        .freq2_hz       (freq2_hz),

        .run            (run),

        .trig_mode      (trig_mode),
        .trig_slope     (trig_slope),
        .single_done    (single_done),

        .off_ch1        (pos_off_ch1),
        .off_ch2        (pos_off_ch2),

        .cap_active     (cap_active),
        .trig_level     (trig_level),
        .trig_hys       (trig_hys),
        .trig_valid     (trig_valid)

    );


    // =========================================================
    // DECIMATION
    // =========================================================

    reg [5:0] decim_cnt;

    reg [9:0] uart_sample;

    reg       send_request;


    always @(posedge CLOCK_50) begin

        if (!rst_n) begin

            decim_cnt    <= 6'd0;

            uart_sample  <= 10'd0;

            send_request <= 1'b0;

        end

        else begin

            send_request <= 1'b0;

            if (adc_valid && !adc_ch) begin

                if (decim_cnt == 6'd35) begin

                    decim_cnt    <= 6'd0;

                    uart_sample  <= adc_data;

                    send_request <= 1'b1;

                end

                else begin

                    decim_cnt <= decim_cnt + 1'b1;

                end

            end

        end

    end


    // =========================================================
    // HEX ASCII
    // =========================================================

    function [7:0] hex_ascii;

        input [3:0] value;

        begin

            if (value < 4'd10)

                hex_ascii = 8'h30 + value;

            else

                hex_ascii = 8'h41 + (value - 4'd10);

        end

    endfunction


    // =========================================================
    // UART FSM
    // =========================================================

    always @(posedge CLOCK_50) begin

        if (!rst_n) begin

            uart_state <= 6'd0;

            uart_data  <= 8'd0;

            uart_valid <= 1'b0;

        end

        else begin

            uart_valid <= 1'b0;

            case (uart_state)


                // =================================================
                // ADC
                // =================================================

                6'd0: begin

                    if (ctrl_pending && !uart_busy) begin

                        uart_data  <= "C";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd18;

                    end

                    else if (send_request && !uart_busy) begin

                        uart_data  <= "A";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd1;

                    end

                end


                6'd1: begin

                    if (uart_busy)

                        uart_state <= 6'd2;

                end


                6'd2: begin

                    if (!uart_busy) begin

                        uart_data  <= "D";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd3;

                    end

                end


                6'd3: begin

                    if (uart_busy)

                        uart_state <= 6'd4;

                end


                6'd4: begin

                    if (!uart_busy) begin

                        uart_data  <= "C";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd5;

                    end

                end


                6'd5: begin

                    if (uart_busy)

                        uart_state <= 6'd6;

                end


                6'd6: begin

                    if (!uart_busy) begin

                        uart_data  <= "=";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd7;

                    end

                end


                6'd7: begin

                    if (uart_busy)

                        uart_state <= 6'd8;

                end


                6'd8: begin

                    if (!uart_busy) begin

                        uart_data <= hex_ascii(
                            {2'b00, uart_sample[9:8]}
                        );

                        uart_valid <= 1'b1;

                        uart_state <= 6'd9;

                    end

                end


                6'd9: begin

                    if (uart_busy)

                        uart_state <= 6'd10;

                end


                6'd10: begin

                    if (!uart_busy) begin

                        uart_data <= hex_ascii(
                            uart_sample[7:4]
                        );

                        uart_valid <= 1'b1;

                        uart_state <= 6'd11;

                    end

                end


                6'd11: begin

                    if (uart_busy)

                        uart_state <= 6'd12;

                end


                6'd12: begin

                    if (!uart_busy) begin

                        uart_data <= hex_ascii(
                            uart_sample[3:0]
                        );

                        uart_valid <= 1'b1;

                        uart_state <= 6'd13;

                    end

                end


                6'd13: begin

                    if (uart_busy)

                        uart_state <= 6'd14;

                end


                6'd14: begin

                    if (!uart_busy) begin

                        uart_data <= 8'h0D;

                        uart_valid <= 1'b1;

                        uart_state <= 6'd15;

                    end

                end


                6'd15: begin

                    if (uart_busy)

                        uart_state <= 6'd16;

                end


                6'd16: begin

                    if (!uart_busy) begin

                        uart_data <= 8'h0A;

                        uart_valid <= 1'b1;

                        uart_state <= 6'd17;

                    end

                end


                6'd17: begin

                    if (!uart_busy)

                        uart_state <= 6'd0;

                end


                // =================================================
                // CTRL
                // =================================================

                6'd18: begin

                    if (uart_busy)

                        uart_state <= 6'd19;

                end


                6'd19: begin

                    if (!uart_busy) begin

                        uart_data <= "T";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd20;

                    end

                end


                6'd20: begin

                    if (uart_busy)

                        uart_state <= 6'd21;

                end


                6'd21: begin

                    if (!uart_busy) begin

                        uart_data <= "R";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd22;

                    end

                end


                6'd22: begin

                    if (uart_busy)

                        uart_state <= 6'd23;

                end


                6'd23: begin

                    if (!uart_busy) begin

                        uart_data <= "L";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd24;

                    end

                end


                6'd24: begin

                    if (uart_busy)

                        uart_state <= 6'd25;

                end


                6'd25: begin

                    if (!uart_busy) begin

                        uart_data <= "=";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd26;

                    end

                end


                6'd26: begin

                    if (uart_busy)

                        uart_state <= 6'd27;

                end


                6'd27: begin

                    if (!uart_busy) begin

                        uart_data <= "T";

                        uart_valid <= 1'b1;

                        uart_state <= 6'd28;

                    end

                end


                6'd28: begin

                    if (uart_busy)

                        uart_state <= 6'd29;

                end


                6'd29: begin

                    if (!uart_busy) begin

                        uart_data <= "0" + ctrl_index;

                        uart_valid <= 1'b1;

                        uart_state <= 6'd30;

                    end

                end


                6'd30: begin

                    if (uart_busy)

                        uart_state <= 6'd31;

                end


                6'd31: begin

                    if (!uart_busy) begin

                        uart_data <= 8'h0D;

                        uart_valid <= 1'b1;

                        uart_state <= 6'd32;

                    end

                end


                6'd32: begin

                    if (uart_busy)

                        uart_state <= 6'd33;

                end


                6'd33: begin

                    if (!uart_busy) begin

                        uart_data <= 8'h0A;

                        uart_valid <= 1'b1;

                        uart_state <= 6'd34;

                    end

                end


                6'd34: begin

                    if (!uart_busy)

                        uart_state <= 6'd0;

                end


                default: begin

                    uart_state <= 6'd0;

                end

            endcase

        end

    end


    // =========================================================
    // UART MODULE
    // =========================================================

    uart_tx #(
        .CLK_FREQ  (50_000_000),
        .BAUD_RATE (115200)
    ) u_uart (

        .clk        (CLOCK_50),
        .rst_n      (rst_n),

        .data_in    (uart_data),
        .data_valid (uart_valid),

        .tx         (UART_TX),
        .tx_busy    (uart_busy)

    );


endmodule