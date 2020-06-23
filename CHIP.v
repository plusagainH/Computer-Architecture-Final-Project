// Your code

module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    wire   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    
    // Todo: any combinational/sequential circuit

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            
        end
        else begin
            PC <= PC_nxt;
            
        end
    end
endmodule

module Control_unit(clk,
                    rst_n,
                    fanin,
                    Branch,
                    MemRead,
                    MemtoReg,
                    ALUOp0,
                    ALUOp1,
                    MemWrite,
                    ALUSrc,
                    RegWrite);
    input       clk,rst_n;
    input [6:0] fanin;
    output      Branch;
    output      MemRead;
    output      MemtoReg;
    output      ALUOp0;
    output      ALUOp1;
    output      MemWrite;
    output      ALUSrc;
    output      RegWrite;
    
    always @(posedge clk or fanin)
        if(!fanin[6]&&fanin[5]&&fanin[4]&&!fanin[3]&&!fanin[2]&&fanin[1]&&fanin[0])begin //add,sub,mul
            
        end
        
        if(!fanin[6]&&!fanin[5]&&!fanin[4]&&!fanin[3]&&!fanin[2]&&fanin[1]&&fanin[0])begin //lw
            Branch = 1'b0;
            MemRead = 1'b1;
            MemtoReg = 1'b1;
            ALUOp0 = 1'b0;
            ALUOp1 = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
        end
        if(!fanin[6]&&fanin[5]&&!fanin[4]&&!fanin[3]&&!fanin[2]&&fanin[1]&&fanin[0])begin //sw
            Branch = 1'b0;
            MemRead = 1'b0;
            ALUOp0 = 1'b0;
            ALUOp1 = 1'b0;
            MemWrite = 1'b1;
            ALUSrc = 1'b1;
            RegWrite = 1'b0;
        end
        if(fanin[6]&&fanin[5]&&!fanin[4]&&!fanin[3]&&!fanin[2]&&fanin[1]&&fanin[0])begin //beq
            Branch = 1'b1;
            MemRead = 1'b0;
            ALUOp0 = 1'b0;
            ALUOp1 = 1'b1;
            MemWrite = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
        end
        if(!fanin[6]&&!fanin[5]&&fanin[4]&&!fanin[3]&&fanin[2]&&fanin[1]&&fanin[0])begin //auipc
            Branch = 1'b0;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            ALUOp0 = 1'b0;
            ALUOp1 = 1'b0;
            MemWrite = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
        end
        if(fanin[6]&&fanin[5]&&!fanin[4]&&fanin[3]&&fanin[2]&&fanin[1]&&fanin[0])begin //jal
            Branch = 1'b1;
            MemRead = 1'b0;
            MemtoReg = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b1;
        end
        if(fanin[6]&&fanin[5]&&!fanin[4]&&!fanin[3]&&fanin[2]&&fanin[1]&&fanin[0])begin //jalr
            Branch = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            RegWrite = 1'b0;
        end
        if(!fanin[6]&&!fanin[5]&&fanin[4]&&!fanin[3]&&!fanin[2]&&fanin[1]&&fanin[0])begin //addi,slti
        
        end
    end

endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module multDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW3

endmodule