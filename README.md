# Grupo SAA0356 - Sistemas Embarcados para Veículos Aéreos

Docente: Glauco Augusto de Paula Caurin
<br>
Integrantes do Grupo:<br>
                      Alice Czyz Froes Fontes - 11802478 <br>
                      Guilherme Azevedo Escudeiro - 11345600 <br>
                      Jorge Henrique Mellega - 11802610 <br>
                      Rhayna Christiani Vasconcelos Marques Casado - 13676429 <br>
                      Jackson Wagner Silva - 12696211 <br>
                      Vitor Marques Delmondes - 10820949 <br>

# Introdução
Este projeto do github é uma fork do projeto original desenvolvido por Henrique Garcia (@griloHBG), disponível em https://github.com/griloHBG/eesc-aero-embedded-systems. Portanto, o objetivo é realizar alterações de forma a corrigir bugs, tornar o software mais fácil de utilizar e expandir suas funcionalidades. Tais objetivos foram propostos pelo docente supracitado, cujo intuito é estimular o desenvolvimento dos alunos envolvidos na área de sistemas embarcados.

Este projeto implementa um sistema de controle baseado em um Controlador dLQR com recursos adicionais para detecção de eventos e modos de operação dinâmicos. Ele é projetado para sistemas mecatrônicos com controle de posição, corrente e velocidade, utilizando a biblioteca Armadillo para cálculos matemáticos e integração com CANopen.

# Recursos do código
  O projeto oferece os seguintes recursos: 
  
1. Modos de Controle:

+ Controle PID.
+ Controle dLQR (Determinístico Linear Quadrático). 
+ Controle dLQR com detecção de eventos baseados em erros dinâmicos.

2. Registro de Dados:

+ Registra informações detalhadas sobre o sistema, como pulsos do motor, corrente real, velocidade e erros de controle.
+ Salva os logs em um arquivo CSV com timestamp e parâmetros de configuração.

3. libraries 

+ Armadillo: cálculos matriciais.
+ chrono: medição de tempo.
+ iostream, fstream: entrada e saída de dados.
+ cmath: cálculos matemáticos.
+ map, vector, tuple: organização de dados.
+ ManoplaLelyBBB: abstração para a interface com dispositivos CANopen (personalizada)

4. Estrutura do código
   
+ Classe MyLog: Gerenciar registros de dados do sistema para exporta no CSV
+ Classe ManoplaLelyBBB: Gerenciar interface entre o programa e os dispositivos CANopen.
+ Estrutura EventDLQRControlPreset: conjunto predefinido de parâmetros para o controle dLQR baseado em eventos.

5. Modos de Operação
   
+ position-mode: Modo de controle baseado em posição.
+ current-mode: Controle de corrente 
+ pid: Controle PID com parâmetros ajustáveis (Kp, Kd, Ki).
+ dlqr e dlqr-event: Controle dLQR com ou sem detecção de eventos.
  
# Quickstart

## Requisitos
Para utilizar o software, espera-se que o usuário tenha instalado em sua máquina as ferramentas git e docker. Para mais informações, consulte as documentações oficiais.

## Compilação para ARM
Primeiramente, realize o clone deste repositório git e seus submódulos:

```bash
git clone --recurse-submodules git@github.com:Jorge-Henrique-Mellega/eesc-aero-embedded-systems-grupoMK.git
```

Navegue para a pasta do software e execute os comandos:

```bash
sudo docker build -f dockerfile/Dockerfile -t myimage .
sudo docker run -it -v .:/project --rm myimage bash
```

Note que "myimage" pode ser alterado para qualquer nome de sua preferência.

Uma vez dentro do container, execute:

```bash 
cd /project
mkdir build_arm
cd build_arm
cmake .. -DARM_TARGET=1
pkg-config --cflags liblely-coapp
pkg-config --libs liblely-coapp
make
```
Seu arquivo executável para ARM estará disponível na pasta build_arm com o nome "eesc-aero-embedded-systems".


