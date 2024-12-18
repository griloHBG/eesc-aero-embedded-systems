# Grupo SAA0356 - Sistemas Embarcados para Veículos Aéreos

Docente: Glauco Augusto de Paula Caurin
<br>
Integrantes do Grupo:<br>
                      Alice Czyz Froes Fontes - 11802478 <br>
                      Jorge Henrique Mellega - 11802610 <br>
                      Rhayna Christiani Vasconcelos Marques Casado - 13676429 <br>
                      Jackson Wagner Silva - 12696211 <br>
                      Vitor Marques Delmondes - 10820949 <br>

# Introdução
Este projeto do github é uma fork do projeto original desenvolvido por Henrique Garcia (@griloHBG), disponível em https://github.com/griloHBG/eesc-aero-embedded-systems. Portanto, o objetivo é realizar alterações de forma a corrigir bugs, tornar o software mais fácil de utilizar e expandir suas funcionalidades. Tais objetivos foram propostos pelo docente supracitado, cujo intuito é estimular o desenvolvimento dos alunos envolvidos na área de sistemas embarcados.

Este projeto implementa diferentes métodos de controle (PID, DLQR e DLQR-Event) para um sistema de controle baseado em CANopen. Ele utiliza as bibliotecas Armadillo e ManoplaLelyBBB para cálculo matricial e comunicação com dispositivos, respectivamente. O código inclui funcionalidades para ajustar os parâmetros do controlador, monitorar o desempenho do sistema e registrar os dados do experimento em arquivos de log.

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


