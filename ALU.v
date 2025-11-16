`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/13/2025 08:22:42 PM
// Design Name: 
// Module Name: alu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`include "defines.vh"
module ALU(
    input   [31:0]  A, B,
    input   [4:0]   ALUCtrl,
    output  [31:0]  ALUResult,
    output          Zero,
    output          LessThan,     // signed comparison
    output          LessThanU     // unsigned comparison
);
    wire [31:0] and_result;
    wire [31:0] or_result;
    wire [31:0] add_result;
    wire [31:0] sub_result;
    wire [31:0] xor_result;
    wire [31:0] lls_result;
    wire [31:0] lrs_result;
    wire [31:0] ars_result;
    wire [31:0] mul_result;
    wire [31:0] div_result;
    wire Cout;
    
    cla cla_inst (
        .a(A),
        .b(B),
        .cin(1'b0),
        .sum(add_result)
    );
    
    mul mul_inst (
        .A(A),
        .B(B),
        .ALUCtrl(ALUCtrl),
        .result(mul_result)
    );
    
    div div_inst (
        .A(A),
        .B(B),
        .ALUCtrl(ALUCtrl),
        .result(div_result)
    );

    assign and_result = A & B;
    assign or_result  = A | B;
    assign xor_result = A ^ B;
    assign {Cout, sub_result} = {1'b0, A} + ~{1'b0, B} + 1'b1;
    assign lls_result = A << B[4:0];
    assign lrs_result = A >> B[4:0];
    assign ars_result = $signed(A) >>> B[4:0];
    assign {Cout, sub_result} = {1'b0, A} + ~{1'b0, B} + 1'b1;
    
    // Unsigned compare
    assign LessThanU = (A < B);   
    
    // Signed compare
    assign LessThan = ($signed(A) < $signed(B));

    assign Zero = (sub_result == 0);

    assign ALUResult = (ALUCtrl == `ALU_AND)           ? and_result  :
                   (ALUCtrl == `ALU_OR)            ? or_result   :
                   (ALUCtrl == `ALU_ADD)           ? add_result  :
                   (ALUCtrl == `ALU_XOR)           ? xor_result  :
                   (ALUCtrl == `ALU_SUB)           ? sub_result  :
                   (ALUCtrl == `ALU_LSHIFT_LEFT)   ? lls_result  :
                   (ALUCtrl == `ALU_LSHIFT_RIGHT)  ? lrs_result  :
                   (ALUCtrl == `ALU_ASHIFT_RIGHT)  ? ars_result  :

                   // *** SLT & SLTU ***
                   (ALUCtrl == `ALU_SLT)           ? {31'b0, LessThan}  :
                   (ALUCtrl == `ALU_SLTU)          ? {31'b0, LessThanU} :

                   // MUL group
                   ((ALUCtrl == `ALU_MUL) ||  
                    (ALUCtrl == `ALU_MULH) ||
                    (ALUCtrl == `ALU_MULSU) || 
                    (ALUCtrl == `ALU_MULU))        ? mul_result :

                   // DIV group
                   ((ALUCtrl == `ALU_DIV)  ||  
                    (ALUCtrl == `ALU_DIVU) ||
                    (ALUCtrl == `ALU_REM)  || 
                    (ALUCtrl == `ALU_REMU))        ? div_result :

                                                       32'hxxxx_xxxx;


endmodule
