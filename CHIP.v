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

    //control unit wire
    wire          branchWire  ;
    wire          memReadWriteWire;
    wire   [1:0]  memtoRegWire;
    wire   [2:0]  aluOpWire   ;
    wire          jalOpWire   ;
    wire          aluSrcWire  ;
    wire          auipcOp0Wire;
    wire          auipcOp1Wire;
    wire   [2:0]  immGenOpWire;
    wire          jalrOpWire  ;
    wire          regWriteWire;

    wire   [31:0] immGenWire  ;  //Imm_Gen output
    wire          zeroWire    ;  //ALU zero
    
    wire   [31:0] aluIn1      ;  //ALU Input 1
    wire   [31:0] aluIn2      ;  //ALU Input 2
    wire   [31:0] aluOut      ;  //ALU OUTPUT

    assign mem_wdata_D = rs2_data;
    assign mem_addr_I = PC ;
    assign mem_addr_D = aluOut;
    assign rs1 = mem_rdata_I[19:15];
    assign rs2 = mem_rdata_I[24:20];
    assign rd = mem_rdata_I[11:7];

    Control_unit control0(
        .opcode(mem_rdata_I[6:0]),
        .funct7(mem_rdata_I[31:25]),
        .funct3(mem_rdata_I[14:12]),
        .Branch(branchWire),
        .MemReadWrite(mem_wen_D),
        .MemtoReg(memtoRegWire),
        .ALUOp(aluOpWire),
        .JALOp(jalOpWire),
        .ALUSrc(aluSrcWire),
        .AUIPCOp0(auipcOp0Wire),
        .AUIPCOp1(auipcOp1Wire),
        .ImmGenOp(immGenOpWire),
        .JALROp(jalrOpWire),
        .RegWrite(regWrite));

    Program_counter counter0(
    	.address(PC),
    	.immGen(immGenWire),
    	.aluResult(aluOut),
    	.address_nxt(PC_nxt),
    	.BranchPC(branchWire),
    	.ZeroPC(zeroWire),
    	.JALOpPC(jalOpWire),
    	.JALROpPC(jalrOpWire));

    Imm_gen imm0(
    	.instruction(mem_rdata_I),
    	.ImmGenOpIG(immGenOpWire),
    	.ImmGenOut(immGenWire));

    Middle_stage middle0(
    	.pc(PC),
    	.rd1(rs1_data),
    	.rd2(rs2_data),
    	.imm(immGenWire),
    	.asrc(aluSrcWire),
    	.auipc0(auipcOp0Wire),
    	.auipc1(auipcOp1Wire),
    	.o1(aluIn1),
    	.o2(aluIn2));

    MemtoReg_mux memtoReg0(
    	.i0(mem_rdata_D),
    	.i1(PC),
    	.i2(aluOut),
    	.memtoReg(memtoRegWire),
    	.writeData(rd_data));

    ALU alu(
        .mode(aluOpWire), 
        .in_A(aluIn1), 
        .in_B(aluIn2), 
        .out(aluOut),
        .zeroALU(zeroWire));

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

module Control_unit(opcode,
                    funct7,
                    funct3,
                    Branch,
                    MemReadWrite,
                    MemtoReg,
                    ALUOp,
                    JALOp,
                    ALUSrc,
                    AUIPCOp0,
                    AUIPCOp1,
                    ImmGenOp,
                    JALROp,
                    RegWrite);
    input [6:0] opcode;
    input [6:0] funct7;
    input [2:0] funct3;
    output reg  Branch;
    output reg  MemReadWrite;
    output reg [1:0] MemtoReg;
    output reg [2:0] ALUOp;
    output reg  JALOp;
    output reg  ALUSrc;
    output reg  AUIPCOp0;
    output reg  AUIPCOp1;
    output reg [2:0] ImmGenOp;
    output reg  JALROp;
    output reg  RegWrite;
    
    always @(opcode or funct7 or funct3) begin
        if(!opcode[6]&&opcode[5]&&opcode[4]&&!opcode[3]&&!opcode[2]&&opcode[1]&&opcode[0])begin //add,sub,mul
            if(funct7[5])begin //sub
                Branch <= 1'b0;
                MemReadWrite <= 1'b0;
                MemtoReg <= 2'b10;
                ALUOp <= 3'b001;
                JALOp <= 1'b0;
                ALUSrc <= 1'b0;
                AUIPCOp0 <= 1'b0;
                AUIPCOp1 <= 1'b0;
                ImmGenOp <= 3'bz;
                JALROp <= 1'b0;
                RegWrite <= 1'b1;
            end
            else begin //add,mul
                if(funct7[0])begin //mul
                    Branch <= 1'b0;
	                MemReadWrite <= 1'b0;
	                MemtoReg <= 2'b10;
                    ALUOp <= 3'b010;
	                JALOp <= 1'b0;
	                ALUSrc <= 1'b0;
	                AUIPCOp0 <= 1'b0;
	                AUIPCOp1 <= 1'b0;
	                ImmGenOp <= 3'bz;
	                JALROp <= 1'b0;
	                RegWrite <= 1'b1;
                end
                else begin //add
                    Branch <= 1'b0;
	                MemReadWrite <= 1'b0;
	                MemtoReg <= 2'b10;
	                ALUOp <= 3'b000;
	                JALOp <= 1'b0;
	                ALUSrc <= 1'b0;
	                AUIPCOp0 <= 1'b0;
	                AUIPCOp1 <= 1'b0;
	                ImmGenOp <= 3'bz;
	                JALROp <= 1'b0;
	                RegWrite <= 1'b1;
                end
            end
        end
        else if(!opcode[6]&&!opcode[5]&&!opcode[4]&&!opcode[3]&&!opcode[2]&&opcode[1]&&opcode[0])begin //lw
            Branch <= 1'b0;
            MemReadWrite <= 1'b0;
            MemtoReg <= 2'b00;
            ALUOp <= 3'b000;
            JALOp <= 1'b0;
            ALUSrc <= 1'b1;
            AUIPCOp0 <= 1'b0;
            AUIPCOp1 <= 1'b0;
            ImmGenOp <= 3'b000;
            JALROp <= 1'b0;
            RegWrite <= 1'b1;
        end
        else if(!opcode[6]&&opcode[5]&&!opcode[4]&&!opcode[3]&&!opcode[2]&&opcode[1]&&opcode[0])begin //sw
            Branch <= 1'b0;
            MemReadWrite <= 1'b1;
            MemtoReg <= 2'bz;
            ALUOp <= 3'b000;
            JALOp <= 1'b0;
            ALUSrc <= 1'b1;
            AUIPCOp0 <= 1'b0;
            AUIPCOp1 <= 1'b0;
            ImmGenOp <= 3'b001;
            JALROp <= 1'b0;
            RegWrite <= 1'b0;
        end
        else if(opcode[6]&&opcode[5]&&!opcode[4]&&!opcode[3]&&!opcode[2]&&opcode[1]&&opcode[0])begin //beq,bne
            if (!funct3[0]) begin  //beq
                Branch <= 1'b1;
                MemReadWrite <= 1'b0;
                MemtoReg <= 2'bz;
                ALUOp <= 3'b001;
                JALOp <= 1'b0;
                ALUSrc <= 1'b0;
                AUIPCOp0 <= 1'b0;
                AUIPCOp1 <= 1'b0;
                ImmGenOp <= 3'b010;
                JALROp <= 1'b0;
                RegWrite <= 1'b0;
            end
            else begin  //bne
                Branch <= 1'b1;
                MemReadWrite <= 1'b0;
                MemtoReg <= 2'bz;
                ALUOp <= 3'b100;
                JALOp <= 1'b0;
                ALUSrc <= 1'b0;
                AUIPCOp0 <= 1'b0;
                AUIPCOp1 <= 1'b0;
                ImmGenOp <= 3'b010;
                JALROp <= 1'b0;
                RegWrite <= 1'b0;
            end
        end
        else if(!opcode[6]&&!opcode[5]&&opcode[4]&&!opcode[3]&&opcode[2]&&opcode[1]&&opcode[0])begin //auipc
            Branch <= 1'b0;
            MemReadWrite <= 1'b0;
            MemtoReg <= 2'b10;
            ALUOp <= 3'b000;
            JALOp <= 1'b0;
            ALUSrc <= 1'b1;
            AUIPCOp0 <= 1'b1;
            AUIPCOp1 <= 1'b1;
            ImmGenOp <= 3'b011;
            JALROp <= 1'b0;
            RegWrite <= 1'b1;
        end
        else if(opcode[6]&&opcode[5]&&!opcode[4]&&opcode[3]&&opcode[2]&&opcode[1]&&opcode[0])begin //jal
            Branch <= 1'b0;
            MemReadWrite <= 1'b0;
            MemtoReg <= 2'b01;
            ALUOp <= 3'bz;
            JALOp <= 1'b1;
            ALUSrc <= 1'bz;
            AUIPCOp0 <= 1'b0;
            AUIPCOp1 <= 1'b0;
            ImmGenOp <= 3'b100;
            JALROp <= 1'b0;
            RegWrite <= 1'b1;
        end
        else if(opcode[6]&&opcode[5]&&!opcode[4]&&!opcode[3]&&opcode[2]&&opcode[1]&&opcode[0])begin //jalr
            Branch <= 1'b0;
            MemReadWrite <= 1'b0;
            MemtoReg <= 2'bz;
            ALUOp <= 3'b000;
            JALOp <= 1'b0;
            ALUSrc <= 1'b1;
            AUIPCOp0 <= 1'b0;
            AUIPCOp1 <= 1'b0;
            ImmGenOp <= 3'b000;
            JALROp <= 1'b1;
            RegWrite <= 1'b0;
        end
        else begin //addi,slti,srli
            if (funct3[2]) begin //srli
                Branch <= 1'b0;
                MemReadWrite <= 1'b0;
                MemtoReg <= 2'b10;
                ALUOp <= 3'b101;
                JALOp <= 1'b0;
                ALUSrc <= 1'b1;
                AUIPCOp0 <= 1'b0;
                AUIPCOp1 <= 1'b0;
                ImmGenOp <= 3'b000;
                JALROp <= 1'b0;
                RegWrite <= 1'b1;
            end
            else if(funct3[1])begin //slti
                Branch <= 1'b0;
	            MemReadWrite <= 1'b0;
	            MemtoReg <= 2'b10;
	            ALUOp <= 3'b011;
	            JALOp <= 1'b0;
	            ALUSrc <= 1'b1;
	            AUIPCOp0 <= 1'b0;
	            AUIPCOp1 <= 1'b0;
	            ImmGenOp <= 3'b000;
	            JALROp <= 1'b0;
	            RegWrite <= 1'b1;
            end
            else begin //addi
                Branch <= 1'b0;
	            MemReadWrite <= 1'b0;
	            MemtoReg <= 2'b10;
	            ALUOp <= 3'b000;
	            JALOp <= 1'b0;
	            ALUSrc <= 1'b1;
	            AUIPCOp0 <= 1'b0;
	            AUIPCOp1 <= 1'b0;
	            ImmGenOp <= 3'b000;
	            JALROp <= 1'b0;
	            RegWrite <= 1'b1;
            end
        end
    end
endmodule

module Program_counter(address, immGen, aluResult, address_nxt, BranchPC, ZeroPC, JALOpPC, JALROpPC);
    input [31:0] address;
    input [31:0] immGen;
    input [31:0] aluResult;
    input BranchPC;
    input ZeroPC;
    input JALOpPC;
    input JALROpPC;
    output reg [31:0] address_nxt;
    always @(address or immGen or aluResult or JALROpPC or BranchPC or ZeroPC or JALOpPC) begin
    	if (!JALROpPC) begin
    		if ((BranchPC && ZeroPC) || JALOpPC)begin
	            address_nxt = address + (immGen << 1);
	        end
	        else begin
	            address_nxt = address + 4;
	        end
    	end
    	else begin
    		address_nxt = aluResult;
    	end
    end
endmodule

module Imm_gen(instruction,ImmGenOpIG,ImmGenOut);
    input [31:0] instruction;
    input [2:0] ImmGenOpIG;
    output reg [31:0] ImmGenOut;
    always @(instruction or ImmGenOpIG) begin
        case(ImmGenOpIG)
            3'b000 : begin  //I-type
                if(instruction[31]==1'b0)begin
                    ImmGenOut[31:12] <= 20'b0000_0000_0000_0000_0000;
                    ImmGenOut[11:0] <= instruction[31:20];
                end
                else begin
                    ImmGenOut[31:12] <= 20'b1111_1111_1111_1111_1111;
                    ImmGenOut[11:0] <= instruction[31:20];
                end
            end
            3'b001 : begin  //S-type
                if(instruction[31]==1'b0)begin
                    ImmGenOut[31:12] <= 20'b0000_0000_0000_0000_0000;
                    ImmGenOut[11:5] <= instruction[31:25];
                    ImmGenOut[4:0] <= instruction[11:7];
                end
                else begin
                    ImmGenOut[31:12] <= 20'b1111_1111_1111_1111_1111;
                    ImmGenOut[11:5] <= instruction[31:25];
                    ImmGenOut[4:0] <= instruction[11:7];
                end
            end
            3'b010 : begin  //B-type
                if(instruction[31]==1'b0)begin
                    ImmGenOut[31:12] <= 20'b0000_0000_0000_0000_0000;
                    ImmGenOut[11] <= instruction[31];
                    ImmGenOut[10] <= instruction[7];
                    ImmGenOut[9:4] <= instruction[30:25];
                    ImmGenOut[3:0] <= instruction[11:8];
                end
                else begin
                    ImmGenOut[31:12] <= 20'b1111_1111_1111_1111_1111;
                    ImmGenOut[11] <= instruction[31];
                    ImmGenOut[10] <= instruction[7];
                    ImmGenOut[9:4] <= instruction[30:25];
                    ImmGenOut[3:0] <= instruction[11:8];
                end
            end
            3'b011 : begin  //U-type
                if(instruction[31]==1'b0)begin
                    ImmGenOut[31:20] <= 12'b0000_0000_0000;
                    ImmGenOut[19:0] <= instruction[31:12];
                end
                else begin
                    ImmGenOut[31:20] <= 12'b1111_1111_1111;
                    ImmGenOut[19:0] <= instruction[31:12];
                end
            end
            3'b100 : begin  //J-type
                if(instruction[31]==1'b0)begin
                    ImmGenOut[31:20] <= 12'b0000_0000_0000;
                    ImmGenOut[19] <= instruction[31];
                    ImmGenOut[18:11] <= instruction[19:12];
                    ImmGenOut[10] <= instruction[20];
                    ImmGenOut[9:0] <= instruction[30:21];
                end
                else begin
                    ImmGenOut[31:20] <= 12'b1111_1111_1111;
                    ImmGenOut[19] <= instruction[31];
                    ImmGenOut[18:11] <= instruction[19:12];
                    ImmGenOut[10] <= instruction[20];
                    ImmGenOut[9:0] <= instruction[30:21];
                end
            end
            default: begin
                ImmGenOut[31:0] <= 32'bz;
            end
        endcase
    end
endmodule

module Middle_stage(pc,rd1,rd2,imm,asrc,auipc0,auipc1,o1,o2);
	input [31:0] pc, rd1, rd2, imm;
	input asrc,auipc0, auipc1;
	output reg [31:0] o1, o2;
	always @(pc or rd1 or rd2 or imm or auipc0 or auipc1) begin
		if (auipc0==1'b1) begin
			o1 = pc;
		end
		else begin
			o1 = rd1;
		end
		if (auipc1==1'b0) begin
			if (asrc==1'b0) begin
				o2 = rd2;
			end
			else begin
				o2 = imm;
			end
		end
		else begin
			o2 = imm << 12;
		end
	end
endmodule

module MemtoReg_mux(i0, i1, i2, memtoReg, writeData);
	input [31:0] i0;
	input [31:0] i1;
	input [31:0] i2;
	input [1:0] memtoReg;
	output reg [31:0] writeData;
	always @(i0 or i1 or i2 or memtoReg) begin
		case (memtoReg)
			2'b00 : writeData <= i0;
			2'b01 : writeData <= i1 + 4;
			2'b10 : writeData <= i2;
			default : writeData <= 31'bz;
		endcase
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

module ALU(mode, in_A, in_B, out, zeroALU);
    // Todo: your HW3
    // Definition of ports
    input  [2:0]  mode;
    input  [31:0] in_A, in_B;
    output [31:0] out;
    output reg zeroALU;

    reg  [63:0] shreg [0:32];
    reg  [32:0] alu_out;

    parameter ADD = 3'b000;
    parameter SUB = 3'b001;
    parameter MULT = 3'b010;
    parameter COMPARE = 3'b011;
    parameter BNE = 3'b100;
    parameter SRLI = 3'b101;

    integer i;

    assign out = shreg[32][31:0]; 

    always @(in_A or in_B or mode) begin
        case(mode)
            ADD:begin
                shreg[32] =  in_A + in_B;
                zeroALU = 1'b0;
            end
            SUB:begin
                shreg[32] =  in_A - in_B;
                if (shreg[32]==64'b0) begin
                    zeroALU = 1'b1;
                end
                else begin
                    zeroALU = 1'b0;
                end
            end
            COMPARE:begin
                shreg[32] =  (in_A<in_B) ? {{63{1'b0}},1'b1} : {64{1'b0}};
                zeroALU = 1'b0;
            end
            MULT:begin
                shreg[0] = in_A;
                for (i=0; i<32; i=i+1) begin
                    if (shreg[0]) begin
                        alu_out = shreg[i][63:32] + in_B;
                        shreg[i+1] = {alu_out, shreg[i][31:1]};
                    end
                    else
                        shreg[i+1] = {1'b0, shreg[i][63:1]};
                end
                zeroALU = 1'b0;
            end
            BNE:begin
                shreg[32] =  in_A - in_B;
                if (shreg[32]==64'b0) begin
                    zeroALU = 1'b0;
                end
                else begin
                    zeroALU = 1'b1;
                end
            end
            SRLI:begin
                zeroALU = 1'b0;
                shreg[32] = in_A >> in_B;
            end
        endcase
    end
endmodule