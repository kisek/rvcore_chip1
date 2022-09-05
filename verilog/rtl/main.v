/********************************************************************************************/
/* 2022-09 Version for chip fabrication, Ver.2022-09-01c                                    */
/* RVCore (RISC-V Core Processor) v2019-10-31a        since  2018-08-07  ArchLab. TokyoTech */
/********************************************************************************************/
`define MEM_SIZE 64*4            // Memory size in Byte
`define TOHOST_ADDR 32'h40008000 // TOHOST_ADDR
`define CMD_PRINT_CHAR 1         //
`define CMD_POWER_OFF  2         //
`define QUEUE_SIZE 32            // queue size of UART fifo
`define SERIAL_WCNT  20          // 100MHz clock and 5Mbaud -> 20
/********************************************************************************************/

`default_nettype none
/********************************************************************************************/
module rvcorep(clk, rst, w_rxd, r_txd, led);
    input  wire       clk, rst;
    input  wire       w_rxd;
    output reg        r_txd;
    output wire [3:0] led;
    
    wire w_clk = clk;

    reg r_rst;
    always @(posedge w_clk) r_rst <= rst;
    
    reg core_rst;
    always @(posedge w_clk) core_rst <= (r_rst | !initdone);

    reg [31:0] r_scnt;
    always @(posedge w_clk) begin
       if(r_rst) r_scnt = 0;
       else  r_scnt <= r_scnt + 1;
    end
    
    reg r_inv;
    always @(posedge w_clk) r_inv <= rst;
    
    assign led = {r_scnt[24:22], r_inv};
    
    /****************************************************************************************/
    reg r_rxd_t1,  r_rxd_t2;
    always@(posedge w_clk) r_rxd_t1 <= w_rxd;
    always@(posedge w_clk) r_rxd_t2 <= r_rxd_t1;
    
    /****************************************************************************************/
    wire [31:0] w_initdata;
    wire [31:0] w_initaddr;
    wire        w_initwe;
    wire        w_initdone;
    PLOADER ploader(w_clk, !r_rst, r_rxd_t2, w_initaddr, w_initdata, w_initwe, w_initdone);

    reg  [31:0] initdata;
    reg  [31:0] initaddr;
    reg  [3:0]  initwe;
    reg         initdone;
    always@(posedge w_clk) begin
        initdata <= (r_rst) ? 0 : (w_initwe) ? w_initdata   : 0;
        initaddr <= (r_rst) ? 0 : (initwe)   ? initaddr + 4 : initaddr;
        initwe   <= (r_rst) ? 0 : {4{w_initwe}};
        initdone <= (r_rst) ? 0 : w_initdone;
    end

    /****************************************************************************************/
    wire        w_halt;
    wire [31:0] w_rout, I_DATA, I_ADDR, D_DATA, WD_DATA, D_ADDR;
    wire [3:0]  D_WE;

    RVCore p(w_clk, !core_rst, w_rout, w_halt, I_ADDR, D_ADDR, I_DATA, D_DATA, WD_DATA, D_WE);

    wire [31:0] tmpdata;
    m_IMEM#(32,`MEM_SIZE/4) imem(w_clk, initwe[0], initaddr[$clog2(`MEM_SIZE)-1:2], 
                                 I_ADDR[$clog2(`MEM_SIZE)-1:2], initdata, I_DATA);
    m_DMEM#(32,`MEM_SIZE/4) dmem(w_clk, core_rst, initwe, initaddr[$clog2(`MEM_SIZE)-1:2], 
                                 initdata, tmpdata, w_clk, !core_rst, D_WE, 
                                 D_ADDR[$clog2(`MEM_SIZE)-1:2], WD_DATA, D_DATA);
                                 
    /****************************************************************************************/
    reg        r_D_ADDR;
    reg        r_D_WE;
    reg [31:0] r_WD_DATA;
    always@(posedge w_clk) begin
        if(r_rst) begin
            r_D_ADDR  <= 0;
            r_D_WE    <= 0;
            r_WD_DATA <= 0;
        end
        else begin
            r_D_ADDR  <= D_ADDR[15] & D_ADDR[30];
            r_D_WE    <= D_WE[0];
            r_WD_DATA <= WD_DATA;
        end
    end

    reg        tohost_we;
    reg [31:0] tohost_data;
    reg [7:0]  tohost_char;
    reg [1:0]  tohost_cmd;
    always@(posedge w_clk) begin
        if(r_rst) begin
            tohost_we   <= 0;
            tohost_data <= 0;
            tohost_char <= 0;
            tohost_cmd  <= 0;           
        end
        else begin
            tohost_we   <= (r_D_ADDR && (r_D_WE));
            tohost_data <= r_WD_DATA;
            tohost_char <= (tohost_we) ? tohost_data[7:0] : 0;
            tohost_cmd  <= (tohost_we) ? tohost_data[17:16] : 0;
        end
    end

    /****************************************************************************************/
    reg [7:0] squeue[0:`QUEUE_SIZE-1];
    reg  [$clog2(`QUEUE_SIZE)-1:0] queue_head = 0;
    reg  [$clog2(`QUEUE_SIZE)-1:0] queue_num  = 0;
    always@(posedge w_clk) begin
        if(r_rst) begin
            queue_head <= 0;
            queue_num <= 0;
        end
        else begin
            if(printchar) squeue[queue_addr] <= tohost_char;
            queue_head <= (!printchar & tx_ready & (queue_num > 0) & !uartwe) ? 
                           queue_head + 1 : queue_head;
            queue_num <= (printchar) ? queue_num + 1 : (tx_ready & (queue_num > 0) & !uartwe)
                          ? queue_num - 1 : queue_num;
        end
    end
    wire [$clog2(`QUEUE_SIZE)-1:0] queue_addr = queue_head+queue_num;
    wire printchar = (tohost_cmd==1);
    
    reg [7:0] uartdata;
    reg       uartwe;
    always@(posedge w_clk) begin
        if(r_rst) begin
            uartdata <= 0;
            uartwe <= 0;
        end
        else begin
            uartdata <= (!printchar & tx_ready & (queue_num > 0) & !uartwe) ? squeue[queue_head] : 0;
            uartwe   <= (!printchar & tx_ready & (queue_num > 0) & !uartwe) ? 1                  : 0;
        end
    end
    
    always@(posedge w_clk) r_txd <= w_txd;
    wire w_txd;
    wire tx_ready;
    UartTx UartTx0(w_clk, !core_rst, uartdata, uartwe, w_txd, tx_ready);
endmodule

/********************************************************************************************/
/* Program Loader: Initialize the main memory, copy memory image to the main memory         */
/********************************************************************************************/
module PLOADER (CLK, RST_X, RXD, ADDR, DATA, WE, DONE);
    input  wire        CLK, RST_X, RXD;
    output reg [31:0]  ADDR;
    output reg [31:0]  DATA;
    output reg         WE;
    output reg         DONE; // program load is done

    reg [31:0] waddr; // memory write address

    wire SER_EN;
    wire [7:0] SER_DATA;
    serialc serc (CLK, RST_X, RXD, SER_DATA, SER_EN);

    always @(posedge CLK) begin
        if(!RST_X) begin
            {ADDR, DATA, WE, waddr, DONE} <= 0;
        end else begin
            if(DONE==0 && SER_EN) begin
                ADDR  <= waddr;
                //ADDR  <= (waddr<32'h40000) ? waddr : {8'h04, 6'd0, waddr[17:0]};
                DATA  <= {SER_DATA, DATA[31:8]};
                WE    <= (waddr[1:0]==3);
                waddr <= waddr + 1;
            end else begin
                WE <= 0;
                if(waddr>=`MEM_SIZE) DONE <= 1;
            end
        end
    end
endmodule

/********************************************************************************************/
module UartTx(CLK, RST_X, DATA, WE, TXD, READY);
    input wire       CLK, RST_X, WE;
    input wire [7:0] DATA;
    output reg       TXD, READY;

    reg [8:0]   cmd;
    reg [11:0]  waitnum;
    reg [3:0]   cnt;

    always @(posedge CLK) begin
        if(!RST_X) begin
            TXD       <= 1'b1;
            READY     <= 1'b1;
            cmd       <= 9'h1ff;
            waitnum   <= 0;
            cnt       <= 0;
        end else if( READY ) begin
            TXD       <= 1'b1;
            waitnum   <= 0;
            if( WE )begin
                READY <= 1'b0;
                cmd   <= {DATA, 1'b0};
                cnt   <= 10;
            end
        end else if( waitnum >= `SERIAL_WCNT ) begin
            TXD       <= cmd[0];
            READY     <= (cnt == 1);
            cmd       <= {1'b1, cmd[8:1]};
            waitnum   <= 1;
            cnt       <= cnt - 1;
        end else begin
            waitnum   <= waitnum + 1;
        end
    end
endmodule

/********************************************************************************************/
/* RS232C serial controller (deserializer):                                                 */
/********************************************************************************************/
`define SS_SER_WAIT  'd0         // RS232C deserializer, State WAIT
`define SS_SER_RCV0  'd1         // RS232C deserializer, State Receive 0th bit
                                 // States Receive 1st bit to 7th bit are not used
`define SS_SER_DONE  'd9         // RS232C deserializer, State DONE
/********************************************************************************************/
module serialc(CLK, RST_X, RXD, DATA, EN);
    input  wire    CLK, RST_X, RXD; // clock, reset, RS232C input
    output [7:0]   DATA;            // 8bit output data
    output reg     EN;              // 8bit output data enable

    reg    [7:0]   DATA;
    reg    [3:0]   stage;
    reg    [12:0]  cnt;             // counter to latch D0, D1, ..., D7
    reg    [11:0]  cnt_start;       // counter to detect the Start Bit
    
    wire   [12:0]  waitcnt;
    assign waitcnt = `SERIAL_WCNT;

    always @(posedge CLK)
      if (!RST_X) cnt_start <= 0;
      else        cnt_start <= (RXD) ? 0 : cnt_start + 1;

    always @(posedge CLK)
      if(!RST_X) begin
          EN     <= 0;
          stage  <= `SS_SER_WAIT;
          cnt    <= 1;
          DATA   <= 0;
      end else if (stage == `SS_SER_WAIT) begin // detect the Start Bit
          EN <= 0;
          stage <= (cnt_start == (waitcnt >> 1)) ? `SS_SER_RCV0 : stage;
      end else begin
          if (cnt != waitcnt) begin
              cnt <= cnt + 1;
              EN <= 0;
          end else begin               // receive 1bit data
              stage  <= (stage == `SS_SER_DONE) ? `SS_SER_WAIT : stage + 1;
              EN     <= (stage == 8)  ? 1 : 0;
              DATA   <= {RXD, DATA[7:1]};
              cnt <= 1;
          end
      end
endmodule

/** some definitions, do not change these values                                           **/
/********************************************************************************************/
`define OPCODE_HALT____ 7'h7F // unique for RVCore
`define ALU_CTRL_ADD___ 4'h0
`define ALU_CTRL_SLL___ 4'h1
`define ALU_CTRL_SLT___ 4'h2
`define ALU_CTRL_SLTU__ 4'h3
`define ALU_CTRL_XOR___ 4'h4
`define ALU_CTRL_SRL___ 4'h5
`define ALU_CTRL_OR____ 4'h6
`define ALU_CTRL_AND___ 4'h7
`define ALU_CTRL_SUB___ 4'h8
`define ALU_CTRL_LUI___ 4'h9
`define ALU_CTRL_AUIPC_ 4'hA
`define ALU_CTRL_JAL___ 4'hB
`define ALU_CTRL_JALR__ 4'hC
`define ALU_CTRL_SRA___ 4'hD
  
/********************************************************************************************/  
module regfile(CLK, rs1, rs2, rdata1, rdata2, WE, rd, wdata);
    input  wire        CLK;
    input  wire [ 4:0] rs1, rs2;
    output wire [31:0] rdata1, rdata2;
    input  wire        WE;
    input  wire [ 4:0] rd;
    input  wire [31:0] wdata;

    reg [31:0] mem [0:31];
    assign rdata1 = (rs1 == 0) ? 0 : (rs1==rd) ? wdata : mem[rs1];
    assign rdata2 = (rs2 == 0) ? 0 : (rs2==rd) ? wdata : mem[rs2];
    always @(posedge CLK) if(WE && (rd!=0)) mem[rd] <= wdata;
endmodule

/********************************************************************************************/
module m_IMEM#(parameter WIDTH=32, ENTRY=256)(CLK, WE, WADDR, RADDR, IDATA, ODATA);
    input  wire                     CLK, WE;
    input  wire [$clog2(ENTRY)-1:0] WADDR;
    input  wire [$clog2(ENTRY)-1:0] RADDR;
    input  wire [WIDTH-1:0]         IDATA;
    output reg  [WIDTH-1:0]         ODATA;
    
    reg [WIDTH-1:0] mem[0:ENTRY-1];
    always @(posedge CLK) begin
        if (WE) mem[WADDR] <= IDATA;
        ODATA <= mem[RADDR];
    end
endmodule

/********************************************************************************************/
module m_DMEM#(parameter WIDTH=32, ENTRY=256)(CLK1, EN1, WE1, ADDR1, IDATA1, ODATA1, 
                                              CLK2, EN2, WE2, ADDR2, IDATA2, ODATA2);
    input  wire                     CLK1, EN1, EN2;
    input  wire [3:0]               WE1;
    input  wire [$clog2(ENTRY)-1:0] ADDR1;
    input  wire [WIDTH-1:0]         IDATA1;
    output reg  [WIDTH-1:0]         ODATA1;
    input  wire                     CLK2;
    input  wire [3:0]               WE2;
    input  wire [$clog2(ENTRY)-1:0] ADDR2;
    input  wire [WIDTH-1:0]         IDATA2;
    output reg  [WIDTH-1:0]         ODATA2;
    
     reg [WIDTH-1:0] mem[0:ENTRY-1];
    always @(posedge CLK1) begin
        if(EN1) begin
            if (WE1[0]) mem[ADDR1][ 7: 0] <= IDATA1[ 7: 0];
            if (WE1[1]) mem[ADDR1][15: 8] <= IDATA1[15: 8];
            if (WE1[2]) mem[ADDR1][23:16] <= IDATA1[23:16];
            if (WE1[3]) mem[ADDR1][31:24] <= IDATA1[31:24];
            ODATA1 <= mem[ADDR1];
        end
    end

    always @(posedge CLK2) begin
        if(EN2) begin
            if (WE2[0]) mem[ADDR2][ 7: 0] <= IDATA2[ 7: 0];
            if (WE2[1]) mem[ADDR2][15: 8] <= IDATA2[15: 8];
            if (WE2[2]) mem[ADDR2][23:16] <= IDATA2[23:16];
            if (WE2[3]) mem[ADDR2][31:24] <= IDATA2[31:24];
            ODATA2 <= mem[ADDR2];
        end
    end
endmodule
  
/********************************************************************************************/
module m_ALU (in1, in2, imm, sel, result, bsel, out);
    input  wire [31:0] in1, in2, imm;
    input  wire [10:0] sel;     // select signal for ALU
    output wire [31:0] result;  // output data of ALU
    input  wire  [6:0] bsel;    // select signal for BRU (Branch Resolution Unit)
    output wire  [6:0] out;     // output data of BRU
    
    wire signed [31:0] sin1 = in1;
    wire signed [31:0] sin2 = in2;

    reg [31: 0] r_result;
    always @(*) begin
        case(sel[3:0])
            1 : r_result =  in1 <  in2;
            2 : r_result = sin1 < sin2;
            3 : r_result = in1 + in2;
            4 : r_result = in1 - in2;
            5 : r_result = in1 ^ in2;
            6 : r_result = in1 | in2;
            7 : r_result = in1 & in2;
            8 : r_result = in1   << in2[4:0];
            9 : r_result = in1   >> in2[4:0];
            10: r_result = sin1 >>> in2[4:0];
            default        : r_result = imm;
        endcase
    end
    assign result = r_result;
   
    wire w_op0 = (bsel[0]  & ( in1 ==  in2));
    wire w_op1 = (bsel[1]  & ( in1 !=  in2));
    wire w_op2 = (bsel[2]  & (sin1 <  sin2));
    wire w_op3 = (bsel[3]  & (sin1 >= sin2));
    wire w_op4 = (bsel[4]  & ( in1 <   in2));
    wire w_op5 = (bsel[5]  & ( in1 >=  in2));
    assign out = {bsel[6], w_op5, w_op4, w_op3, w_op2, w_op1, w_op0};
endmodule

/********************************************************************************************/
module RVCore(CLK, RST_X, r_rout, r_halt, I_ADDR, D_ADDR, I_IN, D_IN, D_OUT, D_WE);
    input  wire        CLK, RST_X;
    output reg  [31:0] r_rout;
    output reg         r_halt;
    output wire [31:0] I_ADDR, D_ADDR;
    input  wire [31:0] I_IN, D_IN;
    output wire [31:0] D_OUT;
    output wire  [3:0] D_WE;

    /**************************** Architecture Register and Pipeline Register ***************/
    reg [17:0] r_pc          ;  // Program Counter
    
    reg        IfId_v        ; ///// IF/ID pipeline register
    reg [31:0] IfId_ir       ; // fetched instruction
    reg [17:0] IfId_pc       ;
    reg [17:0] IfId_pc_n     ;
    reg [ 6:0] IfId_op       ;
    reg [ 2:0] IfId_funct3   ;
    reg [ 4:0] IfId_rs1      ;
    reg [ 4:0] IfId_rs2      ;
    reg [ 4:0] IfId_rd       ;
    reg        IfId_mem_we   ;
    reg        IfId_reg_we   ;
    reg        IfId_op_ld    ;
    reg        IfId_op_im    ;
    reg        IfId_s1       ;
    reg        IfId_s2       ;
    reg [2:0]  IfId_im_s     ;

    reg        IdEx_v        ; ///// ID/EX pipeline register
    reg [17:0] IdEx_pc       ;
    reg [17:0] IdEx_pc_n     ;
    reg [ 6:0] IdEx_op       ;
    reg [ 2:0] IdEx_funct3   ;
    reg [ 4:0] IdEx_rd       ;
    reg [31:0] IdEx_imm      ;
    reg        IdEx_mem_we   ;
    reg        IdEx_reg_we   ;
    reg        IdEx_op_ld    ;
    reg [10:0] IdEx_alu_ctrl ; // Note!!
    reg [ 6:0] IdEx_bru_ctrl ;
    reg [31:0] IdEx_rrs1     ;
    reg [31:0] IdEx_rrs2     ;
    reg [31:0] IdEx_alu_imm  ; // additional value for ALU
    reg [31:0] IdEx_ir       ;
    reg [31:0] IdEx_s1       ;
    reg [31:0] IdEx_s2       ;
    reg [31:0] IdEx_u1       ;
    reg [31:0] IdEx_u2       ;
    reg        IdEx_luse     ; // Note
    reg        IdEx_luse_x   ;
    reg        IdEx_JALR     ;
    reg [2:0]  IdEx_st_we    ;
    
    reg        ExMa_v        ; ///// EX/MA pipeline register
    reg [17:0] ExMa_pc       ;
    reg [17:0] ExMa_pc_n     ;
    reg [ 6:0] ExMa_op       ;
    reg [ 4:0] ExMa_rd       ;
    reg        ExMa_reg_we   ;
    reg        ExMa_op_ld    ;
    reg [31:0] ExMa_rslt     ;
    reg [ 6:0] ExMa_b_rslt   ;
    reg [17:0] ExMa_tkn_pc   ;
    reg [ 1:0] ExMa_addr     ;
    reg        ExMa_b        ;
    reg [31:0] ExMa_wdata    ;
    reg [31:0] ExMa_ir       ;
    reg [17:0] ExMa_npc      ;
    reg [17:0] ExMa_ppc      ;
    reg        ExMa_b_rslt_t1;
    reg        ExMa_b_rslt_t2;
    reg [ 6:0] ExMa_bru_ctrl ;
    reg [ 4:0] ExMa_lds      ; /***** for load instructions *****/
    reg  [2:0] ExMa_funct3   ;
    
    reg        MaWb_v        ; ///// MA/WB pipeline register
    reg [ 4:0] MaWb_rd       ;
    reg [31:0] MaWb_rslt     ;
    reg [17:0] MaWb_pc       ;
    reg [31:0] MaWb_wdata    ;
    reg [31:0] MaWb_ir       ; // just for verify

    /****************************************************************************************/
    reg r_rst;
    always @(posedge CLK) r_rst <= !RST_X | r_halt;

    /**************************** IF  *******************************************************/
    wire [31:0]  w_ir   = I_IN;
    wire w_stall = IdEx_luse;  // stall by load-use (luse) data dependency

    wire w_bmis = ExMa_b & (ExMa_b_rslt ? ExMa_b_rslt_t1 : ExMa_b_rslt_t2);
    wire [17:0] w_pc_true = {((ExMa_b_rslt) ? ExMa_tkn_pc[17:2]: ExMa_npc[17:2]), 2'b00};

    wire [15:0] w_npc = (w_bmis) ? w_pc_true[17:2] : (w_stall) ? r_pc[17:2] : r_pc[17:2]+1;
    
    always @(posedge CLK) if(r_rst) r_pc <= 0; else r_pc <= {w_npc, 2'b00};
    
    assign I_ADDR = {w_npc, 2'b00};

    /****************************************************************************************/
    wire [ 4:0]  If_rd;
    wire [ 4:0]  If_rs1;
    wire [ 4:0]  If_rs2;
    wire [31:0]  w_rrs1;
    wire [31:0]  w_rrs2;
    wire         w_mem_we;
    wire         w_reg_we;
    wire         w_op_ld;
    wire         w_op_im;
    decoder_if dec_if0(w_ir, If_rd, If_rs1, If_rs2, w_mem_we, w_reg_we, w_op_ld, w_op_im);

    always @(posedge CLK) begin
        if(r_rst) begin
            IfId_v        <= 0;
            IfId_mem_we   <= 0;
            IfId_reg_we   <= 0;
            IfId_rd       <= 0;
            IfId_op_ld    <= 0;
            IfId_s1       <= 0;
            IfId_s2       <= 0;
            IfId_rs1      <= 0;
            IfId_rs2      <= 0;
            IfId_op       <= 0;
            IfId_pc       <= 0;
            IfId_pc_n     <= 0;
            IfId_ir       <= 0;
            IfId_funct3   <= 0;
            IfId_op_im    <= 0;
            IfId_im_s     <= 0;          
        end
        else begin
            IfId_v        <= (w_bmis) ? 0 : (w_stall) ? IfId_v      : 1;
            IfId_mem_we   <= (w_bmis) ? 0 : (w_stall) ? IfId_mem_we : w_mem_we;
            IfId_reg_we   <= (w_bmis) ? 0 : (w_stall) ? IfId_reg_we : w_reg_we;
            IfId_rd       <= (w_bmis) ? 0 : (w_stall) ? IfId_rd     : If_rd;
            IfId_op_ld    <= (w_bmis) ? 0 : (w_stall) ? IfId_op_ld  : w_op_ld;
            IfId_s1       <= (w_bmis | w_stall) ? 0 : ((If_rs1==IfId_rd) & IfId_reg_we);
            IfId_s2       <= (w_bmis | w_stall) ? 0 : ((If_rs2==IfId_rd) & IfId_reg_we);
            if(!w_stall) begin
                IfId_rs1    <= If_rs1;
                IfId_rs2    <= If_rs2;
                IfId_op     <= w_ir[6:0];
                IfId_pc     <= r_pc;
                IfId_pc_n   <= {w_npc, 2'b00};
                IfId_ir     <= w_ir;
                IfId_funct3 <= w_ir[14:12];
                IfId_op_im  <= w_op_im;
                IfId_im_s   <= (w_ir[6:2]==5'b01101) ? 3'b001 :    // LUI
                               (w_ir[6:2]==5'b00101) ? 3'b010 :    // AUIPC
                               (w_ir[6:2]==5'b11001) ? 3'b100 :    // JALR
                               (w_ir[6:2]==5'b11011) ? 3'b100 : 0; // JAL
            end
        end
    end

    /**************************** ID  *******************************************************/
    wire [9:0]  Id_alu_ctrl;
    wire [6:0]  Id_bru_ctrl;
    wire [31:0] Id_imm;
    wire [31:0] imm_t;
    decoder_id dec_id0(IfId_ir, Id_alu_ctrl, Id_bru_ctrl, Id_imm);
    
    regfile regs0(CLK, IfId_rs1, IfId_rs2, w_rrs1, w_rrs2, 1'b1, MaWb_rd, MaWb_rslt);
    
    /***** control signal for data forwarding *****/
    wire w_fwd_s1 = (IfId_rs1==ExMa_rd) & (ExMa_reg_we);
    wire w_fwd_s2 = (IfId_rs2==ExMa_rd) & (ExMa_reg_we);

    wire Id_s1 = (IfId_rs1==IdEx_rd) & (IdEx_reg_we);
    wire Id_s2 = (IfId_rs2==IdEx_rd) & (IdEx_reg_we);
         
    wire Id_luse = r_rst | !IdEx_luse &
         (IfId_op_ld) & ((w_ir[19:15]==IfId_rd) | (w_ir[24:20]==IfId_rd));
                   // Note: this condition of load-use may gererate false dependency
                   
    always @(posedge CLK) begin
        if(r_rst) begin
            IdEx_v        <= 0;
            IdEx_op_ld    <= 0;
            IdEx_mem_we   <= 0;
            IdEx_reg_we   <= 0;
            IdEx_luse     <= 1;
            IdEx_luse_x   <= 0;
            IdEx_op       <= 0;
            IdEx_pc       <= 0;
            IdEx_pc_n     <= 0;
            IdEx_rd       <= 0;
            IdEx_ir       <= 0;
            IdEx_funct3   <= 0;
            IdEx_imm      <= 0;
            IdEx_alu_ctrl <= 0;
            IdEx_bru_ctrl <= 0;
            IdEx_JALR     <= 0;
            IdEx_st_we[0] <= 0;
            IdEx_st_we[1] <= 0;
            IdEx_st_we[2] <= 0;
            IdEx_s1       <= 0;
            IdEx_s2       <= 0;
            IdEx_u1       <= 0;
            IdEx_u2       <= 0;
            IdEx_alu_imm  <= 0;
            IdEx_rrs1     <= 0;
            IdEx_rrs2     <= 0;
        end
        else begin
            IdEx_v        <= (w_bmis | w_stall) ? 0 : IfId_v;
            IdEx_op_ld    <= (w_bmis | w_stall) ? 0 : IfId_op_ld;
            IdEx_mem_we   <= (w_bmis | w_stall) ? 0 :IfId_mem_we;
            IdEx_reg_we   <= (w_bmis | w_stall) ? 0 :IfId_reg_we;
            IdEx_luse     <= Id_luse;
            IdEx_luse_x   <= !Id_luse;
            IdEx_op       <= IfId_op;
            IdEx_pc       <= IfId_pc;
            IdEx_pc_n     <= IfId_pc_n;
            IdEx_rd       <= IfId_rd;
            IdEx_ir       <= IfId_ir;
            IdEx_funct3   <= IfId_funct3;
            IdEx_imm      <= Id_imm;
            IdEx_alu_ctrl <= {(Id_alu_ctrl[9] |Id_alu_ctrl[8]), Id_alu_ctrl};
            IdEx_bru_ctrl <= Id_bru_ctrl;
            IdEx_JALR     <= (IfId_op[6:2]==5'b11001);
            IdEx_st_we[0] <= ((w_bmis | w_stall) ? 0 :IfId_mem_we) & (IfId_funct3[1:0]==0);
            IdEx_st_we[1] <= ((w_bmis | w_stall) ? 0 :IfId_mem_we) & IfId_funct3[0];
            IdEx_st_we[2] <= ((w_bmis | w_stall) ? 0 :IfId_mem_we) & IfId_funct3[1];
            IdEx_s1       <= {32{Id_s1}};
            IdEx_s2       <= {32{Id_s2}};
            IdEx_u1       <= {32{!Id_s1 & w_fwd_s1}};
            IdEx_u2       <= {32{!Id_s2 & w_fwd_s2}};
            IdEx_alu_imm  <= (IfId_im_s[0]) ? {IfId_ir[31:12], 12'b0}           :   
                             (IfId_im_s[1]) ? IfId_pc + {IfId_ir[31:12], 12'b0} :   
                             (IfId_im_s[2]) ? IfId_pc + 4                       : 0;
            IdEx_rrs1     <= (Id_s1 | w_fwd_s1) ? 0 : w_rrs1;
            IdEx_rrs2     <= (Id_s2 | w_fwd_s2) ? 0 : (IfId_op_im) ? Id_imm : w_rrs2;
        end
    end
    
    /**************************** EX  *******************************************************/
    wire [31:0] Ex_rrs1 = ((IdEx_s1) & ExMa_rslt) ^ ((IdEx_u1) & MaWb_rslt) ^ IdEx_rrs1;
    wire [31:0] Ex_rrs2 = ((IdEx_s2) & ExMa_rslt) ^ ((IdEx_u2) & MaWb_rslt) ^ IdEx_rrs2;
    
    wire    [6:0]  w_b_rslt;  // BRU result
    wire    [31:0] w_a_rslt;  // ALU result
    wire    [17:0] w_tkn_pc;  // Taken PC

    assign w_tkn_pc = (IdEx_JALR) ? Ex_rrs1+IdEx_imm : IdEx_pc+IdEx_imm; // using rrs1
    assign D_ADDR = Ex_rrs1 + IdEx_imm;                                  // using rrs1
    assign D_OUT  = (IdEx_funct3[0]) ? {2{Ex_rrs2[15:0]}} :              // using rrs2
                    (IdEx_funct3[1]) ? Ex_rrs2 : {4{Ex_rrs2[7:0]}};      // using rrs2
    
    m_ALU alu0(Ex_rrs1, Ex_rrs2, IdEx_alu_imm, IdEx_alu_ctrl, w_a_rslt, 
               IdEx_bru_ctrl, w_b_rslt);

    always @(posedge CLK) begin
        if(r_rst) begin
            ExMa_v        <= 0;
            ExMa_reg_we   <= 0;
            ExMa_b        <= 0;
            ExMa_rslt     <= 0;
            ExMa_b_rslt   <= 0;
            ExMa_ir       <= 0;
            ExMa_pc       <= 0;
            ExMa_ppc      <= 0;
            ExMa_npc      <= 0;
            ExMa_pc_n     <= 0;
            ExMa_tkn_pc   <= 0;
            ExMa_op       <= 0;
            ExMa_rd       <= 0;
            ExMa_op_ld    <= 0;
            ExMa_addr     <= 0;
            ExMa_wdata    <= 0;
            ExMa_b_rslt_t1<= 0;
            ExMa_b_rslt_t2<= 0;
            ExMa_bru_ctrl <= 0;
            ExMa_funct3   <= 0;
        end
            else begin
            ExMa_v        <= (w_bmis) ? 0 : IdEx_v;
            ExMa_reg_we   <= (w_bmis) ? 0 : IdEx_reg_we;
            ExMa_b        <= (!RST_X || w_bmis || IdEx_v==0) ? 0 : (IdEx_bru_ctrl!=0);
            ExMa_rslt     <= w_a_rslt;
            ExMa_b_rslt   <= w_b_rslt;
            ExMa_ir       <= IdEx_ir;
            ExMa_pc       <= IdEx_pc;   // pc of this instruction
            ExMa_ppc      <= IdEx_pc-4;
            ExMa_npc      <= IdEx_pc+4; // next pc
            ExMa_pc_n     <= IdEx_pc_n; // predicted_next pc
            ExMa_tkn_pc   <= w_tkn_pc;  // taken pc
            ExMa_op       <= IdEx_op;
            ExMa_rd       <= IdEx_rd;
            ExMa_op_ld    <= IdEx_op_ld;
            ExMa_addr     <= D_ADDR[1:0];
            ExMa_wdata    <= D_OUT;
            ExMa_b_rslt_t1<= (w_tkn_pc   !=IdEx_pc_n); // to detect branch pred miss
            ExMa_b_rslt_t2<= ((IdEx_pc+4)!=IdEx_pc_n); // to detect branch pred miss
            ExMa_bru_ctrl <= IdEx_bru_ctrl;
            ExMa_funct3   <= IdEx_funct3;
        end
    end

    /***** for store instruction *****/
    wire [3:0] w_we_sb = (IdEx_st_we[0]) ? (4'b0001 << D_ADDR[1:0])       : 0;
    wire [3:0] w_we_sh = (IdEx_st_we[1]) ? (4'b0011 << {D_ADDR[1], 1'b0}) : 0;
    wire [3:0] w_we_sw = (IdEx_st_we[2]) ? 4'b1111                        : 0;
    assign D_WE = {4{!w_bmis}} & (w_we_sh ^ w_we_sw ^ w_we_sb);
    
    always @(posedge CLK) begin
        if(r_rst) ExMa_lds <= 0;
        else ExMa_lds <= (!IdEx_op_ld) ? 0 :
                        (IdEx_funct3==3'b000) ? 5'b01001 :
                        (IdEx_funct3==3'b001) ? 5'b01010 :
                        (IdEx_funct3==3'b010) ? 5'b00100 :
                        (IdEx_funct3==3'b100) ? 5'b00001 :
                                                5'b00010 ;
    end
    
    /**************************** MEM *******************************************************/
    wire [1:0]  w_adr  = ExMa_addr;
    wire [7:0]  w_lb_t = D_IN >> ({w_adr, 3'd0});
    wire [15:0] w_lh_t = (w_adr[1]) ? D_IN[31:16] : D_IN[15:0];

    wire [31:0] w_ld_lb  = {{24{w_lb_t[7]}},  w_lb_t[7:0]};
    wire [31:0] w_ld_lbu = {24'd0,            w_lb_t[7:0]};
    wire [31:0] w_ld_lh  = {{16{w_lh_t[15]}}, w_lh_t[15:0]};
    wire [31:0] w_ld_lhu = {16'd0,            w_lh_t[15:0]};
    wire [31:0] w_ld_lw  = D_IN;

    reg [31:0] r_ld=0;
    always @(*) begin
        case(ExMa_funct3)
            3'b000 : r_ld = w_ld_lb;
            3'b001 : r_ld = w_ld_lh;
            3'b010 : r_ld = w_ld_lw;
            3'b100 : r_ld = w_ld_lbu;
            3'b101 : r_ld = w_ld_lhu;
            default: r_ld = 0;
        endcase
    end
    wire [31:0] Ma_rslt = (ExMa_op_ld) ? r_ld : ExMa_rslt;
    
    always @(posedge CLK) begin
        if(r_rst) begin
            MaWb_v     <= 0;
            MaWb_pc    <= 0;
            MaWb_ir    <= 0;
            MaWb_wdata <= 0;
            MaWb_rd    <= 0;
            MaWb_rslt  <= 0;           
        end
        else begin
            MaWb_v     <= ExMa_v;
            MaWb_pc    <= ExMa_pc;
            MaWb_ir    <= ExMa_ir;
            MaWb_wdata <= ExMa_wdata;
            MaWb_rd    <= (ExMa_v) ? ExMa_rd : 0;
            MaWb_rslt  <= Ma_rslt;
        end
    end

    /**************************** others ****************************************************/
    always @(posedge CLK) begin
        if(r_rst) r_halt <= 0;
        else if (ExMa_op==`OPCODE_HALT____) r_halt <= 1; /// Note
    end
    
    always @(posedge CLK) begin
        if(r_rst) r_rout <= 0;
        else r_rout <= (MaWb_v) ? MaWb_pc : r_rout; /// Note
    end
endmodule

/***** Instraction decoder, see RV32I opcode map                                        *****/
/********************************************************************************************/
module decoder_id(ir, alu_ctrl, bru_ctrl, imm);
    input  wire [31:0] ir;
    output reg  [ 9:0] alu_ctrl;
    output reg  [ 6:0] bru_ctrl;
    output wire [31:0] imm;
    
    wire [4:0] op     = ir[ 6: 2]; // use 5-bit, cause lower 2-bit are always 2'b11
    wire [2:0] funct3 = ir[14:12];
    wire [6:0] funct7 = ir[31:25];

    wire r_type = (op==5'b01100);
    wire s_type = (op[4:2]==3'b010); // (op==5'b01000);
    wire b_type = (op==5'b11000);
    wire j_type = (op==5'b11011);
    wire u_type = ({op[4], op[2:0]} ==4'b0101);
    wire i_type = (op==5'b11001 || op==5'b00000 || op==5'b00100);

    wire [31:0] imm_U = (u_type) ? {ir[31:12], 12'b0} : 0;
    wire [31:0] imm_I = (i_type) ? {{21{ir[31]}}, ir[30:20]} : 0;
    wire [31:0] imm_S = (s_type) ? {{21{ir[31]}}, ir[30:25], ir[11:7]} : 0;
    wire [31:0] imm_B = (b_type) ? {{20{ir[31]}}, ir[7], ir[30:25] ,ir[11:8], 1'b0} : 0;
    wire [31:0] imm_J = (j_type) ? {{12{ir[31]}}, ir[19:12], ir[20], ir[30:21], 1'b0} : 0;
    assign imm = imm_U ^ imm_I ^ imm_S ^ imm_B ^ imm_J;

    reg [3:0] r_alu_ctrl;
    always @(*) begin
        case(op)
            5'b01100 : r_alu_ctrl = {funct7[5], funct3}; 
            5'b00100 : r_alu_ctrl = (funct3==3'h5) ? {funct7[5], funct3} : {1'b0, funct3};
            default  : r_alu_ctrl = 4'b1111;
        endcase
    end

    always @(*) begin
        case(r_alu_ctrl)
            `ALU_CTRL_SLTU__ : alu_ctrl = 1;
            `ALU_CTRL_SLT___ : alu_ctrl = 2;
            `ALU_CTRL_ADD___ : alu_ctrl = 3;
            `ALU_CTRL_SUB___ : alu_ctrl = 4;
            `ALU_CTRL_XOR___ : alu_ctrl = 5;
            `ALU_CTRL_OR____ : alu_ctrl = 6;
            `ALU_CTRL_AND___ : alu_ctrl = 7;
            `ALU_CTRL_SLL___ : alu_ctrl = 8;
            `ALU_CTRL_SRL___ : alu_ctrl = 9;
            `ALU_CTRL_SRA___ : alu_ctrl = 10;
            default          : alu_ctrl = 0;
        endcase
    end
    
    always @(*) begin /***** one-hot encoding *****/
        case(op)
            5'b11011 : bru_ctrl =                    7'b1000000;     // JAL  -> taken
            5'b11001 : bru_ctrl =                    7'b1000000;     // JALR -> taken
            5'b11000 : bru_ctrl = (funct3==3'b000) ? 7'b0000001 :    // BEQ
                                  (funct3==3'b001) ? 7'b0000010 :    // BNE
                                  (funct3==3'b100) ? 7'b0000100 :    // BLT
                                  (funct3==3'b101) ? 7'b0001000 :    // BGE
                                  (funct3==3'b110) ? 7'b0010000 :    // BLTU
                                  (funct3==3'b111) ? 7'b0100000 : 0; // BGEU
            default : bru_ctrl = 0;
        endcase
    end
endmodule

/***** Instraction decoder, see RV32I opcode map                                        *****/
/********************************************************************************************/
module decoder_if(ir, rd, rs1, rs2, mem_we, reg_we, op_ld, op_imm);
    input  wire [31:0] ir;
    output wire [ 4:0] rd, rs1, rs2;
    output wire        mem_we, reg_we, op_ld, op_imm;
    
    wire [4:0] op = ir[ 6: 2];
    wire r_type = (op==5'b01100);
    wire s_type = (op[4:2]==3'b010); // (op==5'b01000);
    wire b_type = (op==5'b11000);
    wire j_type = (op==5'b11011);
    wire u_type = ({op[4], op[2:0]} ==4'b0101);
    wire i_type = (op==5'b11001 || op==5'b00000 || op==5'b00100);

    assign reg_we = (ir[11:7]!=0) & (op[3:0]!=4'b1000);  //!s_type && !b_type;
    assign mem_we = s_type;
    assign op_ld  = (op==5'b00000);
    assign op_imm = (op==5'b00100);
    assign rd     = (reg_we) ? ir[11:7] : 5'd0;
    assign rs1    = ir[19:15]; // (!u_type && !j_type)       ? ir[19:15] : 5'd0;
    assign rs2    = (!op_imm) ? ir[24:20] : 5'd0;
endmodule

/********************************************************************************************/
`default_nettype wire
/********************************************************************************************/
