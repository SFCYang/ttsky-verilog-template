`timescale 1ns / 1ps

// Aplicación Sistemas Digitales PUC 2025-1 - Naomi Aldea
// ----------------------------
// Semáforo con contador integrado (Verilog-2001 compatible)
// ----------------------------
module tt_um_semaforo(
    input  clk,
    input  rst,
    input  TA, TB,       // sensores de vehículos
    input  P, R,         // botones desfile / reset desfile
    output reg [1:0] LA, LB, // semáforos (00=Rojo, 01=Amarillo, 10=Verde)
    output reg on            // señal del fin de tiempo del contador
);

    // ----- Parámetros y registros internos -----
    localparam S0       = 3'd0;
    localparam S1       = 3'd1;
    localparam S2       = 3'd2;
    localparam S3       = 3'd3;
    localparam S_PARADE = 3'd4;

    reg [2:0] state, next_state;
    reg M;   // modo: 1=normal, 0=desfile
    reg En;  // habilita contador

    // ----- Registro de estado -----
    always @(posedge clk or posedge rst) begin
        if (rst)
            state <= S0;
        else
            state <= next_state;
    end

    // ----- Registro de modo -----
    always @(posedge clk or posedge rst) begin
        if (rst)
            M <= 1;      // modo normal
        else if (P)
            M <= 0;      // activar desfile
        else if (R)
            M <= 1;      // salir de desfile
    end

    // ----- Lógica de siguiente estado -----
    always @(*) begin
        next_state = state;
        case (state)
            S0: begin // Avenida A verde
                if (!TA && M) next_state = S1;
                else if (!M) next_state = S_PARADE;
            end
            S1: begin // Avenida A amarillo
                if (on) next_state = S2;
            end
            S2: begin // Avenida B verde
                if (!TB && M) next_state = S3;
                else if (!M) next_state = S_PARADE;
            end
            S3: begin // Avenida B amarillo
                if (on) next_state = S0;
            end
            S_PARADE: begin
                if (R) next_state = S0;
            end
            default: next_state = S0;
        endcase
    end

    // ----- Salidas -----
    always @(*) begin
        // habilita contador solo en estados amarillos
        En = (state == S1 || state == S3);

        case (state)
            S0: begin LA=2'b10; LB=2'b00; end // A verde, B rojo
            S1: begin LA=2'b01; LB=2'b00; end // A amarillo, B rojo
            S2: begin LA=2'b00; LB=2'b10; end // A rojo, B verde
            S3: begin LA=2'b00; LB=2'b01; end // A rojo, B amarillo
            S_PARADE: begin LA=2'b00; LB=2'b10; end // B verde fijo
            default: begin LA=2'b00; LB=2'b00; end
        endcase
    end

    // ----------------------------
    // Contador integrado
    // ----------------------------
    parameter WIDTH = 8;   // ancho del registro
    parameter VALUE = 20;  // valor máximo
    reg [WIDTH-1:0] Q;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            Q  <= 0;
            on <= 0;
        end else if (!En) begin
            Q  <= 0;
            on <= 0;
        end else if (Q >= VALUE) begin
            Q  <= VALUE;
            on <= 1;
        end else begin
            Q  <= Q + 1;
            on <= 0;
        end
    end

endmodule
