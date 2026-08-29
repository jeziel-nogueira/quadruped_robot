# 🐕 Robô Quadrúpede 12-DoF — Locomoção CPG & Dashboard Web

Sistema completo de controle de locomoção para robô quadrúpede de 12 Graus de Liberdade (12-DoF) baseado em **Geradores de Padrões Centrais (CPG - Central Pattern Generator)**, integrando controle em tempo real via **Raspberry Pi**, feedback sensorial (IMU + Ultrassônico) e uma interface web moderna para telemetria, sintonia dinâmica e calibração de servomotores.

---

## 📋 Índice

- [Visão Geral](#-visão-geral)
- [Arquitetura do Sistema](#-arquitetura-do-sistema)
- [Estrutura do Projeto](#-estrutura-do-projeto)
- [Módulo do Robô (Backend & Controle)](#-módulo-do-robô-backend--controle)
  - [Controlador CPG & Cinemática](#controlador-cpg--cinemática)
  - [Hardware & Sensores](#hardware--sensores)
  - [Modo Simulação (Mock) vs Hardware Real](#modo-simulação-mock-vs-hardware-real)
  - [Ferramentas de Calibração](#ferramentas-de-calibração)
- [Módulo Web (Frontend & Telemetria)](#-módulo-web-frontend--telemetria)
  - [Dashboard de Controle (`index.html`)](#dashboard-de-controle-indexhtml)
  - [Estúdio de Calibração Web (`calibrate.html`)](#estúdio-de-calibração-web-calibratehtml)
- [Pinout e Conexões de Hardware](#-pinout-e-conexões-de-hardware)
- [Guia de Instalação e Execução](#-guia-de-instalação-e-execução)
  - [1. Configuração do Ambiente Python](#1-configuração-do-ambiente-python)
  - [2. Executando em Modo Simulação (PC / Windows / Linux)](#2-executando-em-modo-simulação-pc--windows--linux)
  - [3. Executando no Raspberry Pi (Hardware Real)](#3-executando-no-raspberry-pi-hardware-real)
  - [4. Desenvolvimento do Frontend Web (Opcional)](#4-desenvolvimento-do-frontend-web-opcional)
- [Protocolo de Comunicação & APIs](#-protocolo-de-comunicação--apis)
- [Arquivo de Configuração (`config.json`)](#-arquivo-de-configuração-configjson)

---

## 🌟 Visão Geral

Este projeto combina robótica bioinspirada e engenharia de software moderna:
- **Locomoção Bioinspirada**: Redes de osciladores neurais não-lineares acoplados (Matsuoka CPG) geram padrões de marcha coordenados e adaptativos para as 4 patas.
- **Estabilização Ativa**: Leitura de orientação em tempo real (Roll/Pitch/Yaw) via IMU MPU-6050 para manter o equilíbrio e evitar capotamento.
- **Desvio de Obstáculos**: Sensor ultrassônico HC-SR04 modulariza a velocidade e navegação ao detectar obstáculos à frente.
- **Telemetria em Alta Frequência (50 Hz)**: Servidor FastAPI com WebSockets transmite estados articulares, atitude e sensores com baixíssima latência.
- **Interface Web Reativa**: Painel de controle responsivo com gráficos ao vivo, visualizador de atitude, joystick virtual e ajuste fino de ganhos em tempo de execução.

---

## 🏗️ Arquitetura do Sistema

```mermaid
graph TD
    subgraph Frontend ["🌐 Interface Web (HTML5 / Vanilla JS / CSS3)"]
        UI_Dash["Dashboard (index.html)<br/>Telemetria, Gráficos, Joystick, CPG"]
        UI_Calib["Calibração (calibrate.html)<br/>Ajuste de Servos, Trims, Testes"]
    end

    subgraph Backend ["⚙️ Servidor & Controle (Python / FastAPI / CPG)"]
        WS_Server["FastAPI / Uvicorn Server<br/>WebSockets (50Hz) + REST API"]
        CPG_Core["CPG Solver (CPGNetwork)<br/>Osciladores Matsuoka + Feedback Loop"]
        Kinematics["Cinemática & Mapeamento<br/>12 Articulações (Yaw/Pitch/Knee)"]
    end

    subgraph HardwareLayer ["🔌 Camada de Hardware (Raspberry Pi / Simulação)"]
        PCA["Driver PCA9685 (I2C)<br/>12 Servos PWM"]
        IMU["Sensor IMU MPU-6050 (I2C)<br/>Filtro Complementar"]
        SONAR["Sensor Ultrassônico HC-SR04<br/>GPIO Echo/Trigger"]
        MOCK["MockHardware<br/>Simulador Matemático para PC"]
    end

    UI_Dash <-->|WebSocket /ws (50Hz)| WS_Server
    UI_Calib <-->|REST API /api/servo/*| WS_Server
    WS_Server <--> CPG_Core
    CPG_Core --> Kinematics
    Kinematics --> PCA
    Kinematics -.-> MOCK
    IMU --> CPG_Core
    SONAR --> CPG_Core
```

---

## 📁 Estrutura do Projeto

```text
quadruped_robot/
├── main.py                    # Ponto de entrada raiz da aplicação
├── requirements.txt           # Dependências Python (PC e Raspberry Pi)
├── README.md                  # Documentação completa do projeto
│
├── robot/                     # 🤖 Backend do Robô em Python
│   ├── main.py                # Loop principal de controle (50 Hz), I/O e telemetria
│   ├── config.json            # Configuração persistida de CPG, offsets e calibração de servos
│   │
│   ├── controllers/           # Algoritmos de locomoção e controle
│   │   ├── cpg_network.py     # Solucionador CPG (redes de osciladores acoplados)
│   │   └── kinematics.py      # Mapeamento cinemático e conversão de ângulos
│   │
│   ├── hardware/              # Abstração e drivers de sensores e atuadores
│   │   ├── interface.py       # Interfaces base (RobotHardware, MockHardware)
│   │   ├── physical.py        # Agregador de hardware real para Raspberry Pi (PiHardware)
│   │   ├── servo_driver.py    # Driver I2C do PCA9685 e calibração dos 12 servos
│   │   ├── imu_sensor.py      # Driver I2C do MPU-6050 com filtro complementar
│   │   └── range_sensor.py    # Driver GPIO do sensor ultrassônico HC-SR04
│   │
│   ├── telemetry/             # Servidor de comunicação e APIs
│   │   └── server.py          # Servidor FastAPI com WebSockets, rotas REST e arquivos estáticos
│   │
│   └── tools/                 # Utilitários de linha de comando
│       └── calibrate_servos.py # Ferramenta CLI interativa de calibração de servos
│
└── web/                       # 🌐 Frontend Web (Dashboard & Calibração)
    ├── index.html             # Painel de controle e telemetria em tempo real
    ├── calibrate.html         # Painel gráfico de calibração individual dos servos
    ├── package.json           # Configuração de scripts de desenvolvimento (Vite)
    ├── css/
    │   └── style.css          # Estilização moderna em tema escuro com glassmorphism
    └── js/
        ├── app.js             # Lógica do dashboard, joystick virtual e envio de comandos
        ├── chart.js           # Renderizador de gráficos e osciloscópio de juntas
        └── websocket.js       # Gerenciador de conexão WebSocket com auto-reconnect
```

---

## 🤖 Módulo do Robô (Backend & Controle)

### Controlador CPG & Cinemática
A locomoção do robô é gerada através de uma rede de osciladores não-lineares acoplados:
- **Oscilador Master (Hip Yaw / Hip Pitch)**: Determina a cadência e avanço/recuo da pata.
- **Oscilador Slave (Knee)**: Controla a flexão/extensão do joelho para levantar a pata na fase de balanço.
- **Matriz de Acoplamento**: Define a defasagem entre as quatro patas para marchas quadrupedais (trot, walk, etc.).
- **Malha de Realimentação (Sensory Feedback)**:
  - **Correção de Yaw ($K_y$)**: Compensa desvios direcionais.
  - **Correção de Pitch ($K_p$)**: Ajusta a amplitude das patas dianteiras/traseiras em aclives/declives.
  - **Desaceleração por Distância ($K_u$)**: Reduz amplitude e frequência ao aproximar-se de obstáculos detectados pelo ultrassom.

### Hardware & Sensores
1. **Atuadores (12 Servos PWM)**: 3 juntas por pata (Pata Dianteira Esquerda `FL`, Dianteira Direita `FR`, Traseira Esquerda `HL`, Traseira Direita `HR`).
2. **Driver PWM PCA9685**: Controla os 12 canais via barramento I2C com frequência de 50 Hz e resolução de 12 bits (0 a 4095 ticks).
3. **IMU MPU-6050**: Fornece giroscópio e acelerômetro de 3 eixos. O código executa um filtro complementar em tempo real para estimar Roll, Pitch e Yaw.
4. **Ultrassônico HC-SR04**: Mede distância frontal por pulsos ultrassônicos (Trigger/Echo).

### Modo Simulação (Mock) vs Hardware Real
- **No Windows / PC**: O sistema detecta automaticamente o sistema operacional e ativa o `MockHardware`. O gerador de dados simula a física, atitude e leitura de distância sem necessidade de hardware físico.
- **No Raspberry Pi**: O sistema inicializa o `PiHardware`, comunicando-se com os dispositivos I2C e pinos GPIO reais.

### Ferramentas de Calibração
O projeto inclui a ferramenta interativa de linha de comando `robot/tools/calibrate_servos.py`:
```bash
# Menu interativo no terminal
python -m robot.tools.calibrate_servos

# Comandos diretos:
python -m robot.tools.calibrate_servos --center   # Move todos os servos para 90°
python -m robot.tools.calibrate_servos --sweep    # Varre 0° -> 180° -> 0° em todos os servos
python -m robot.tools.calibrate_servos --stand    # Move para pose de pé pré-definida
```

---

## 🌐 Módulo Web (Frontend & Telemetria)

O frontend é construído em Vanilla HTML5, CSS3 e JavaScript modular, servido diretamente pelo backend FastAPI.

### Dashboard de Controle (`index.html`)
- **Status & Conexão**: Indicador de conexão WebSocket com auto-reconectar e latência.
- **Monitor de Atitude 3D / Nível**: Visualização gráfica dos ângulos de Roll, Pitch e Yaw da IMU.
- **Osciloscópio de Juntas**: Gráficos dinâmicos em Canvas/Chart com as posições angulares em tempo real das 12 articulações.
- **Joystick & Controle de Marcha**:
  - Habilitar/Desabilitar geração de marcha (Start/Stop).
  - Joystick virtual para controle de direção (Curva Esquerda / Direita).
- **Ajuste Fino de Parâmetros CPG**: Sliders e campos numéricos para alterar $\tau_m$, ganhos de feedback ($K_y, K_p, K_u$), fatores de acoplamento e escalas em tempo de execução sem reiniciar o robô.

### Estúdio de Calibração Web (`calibrate.html`)
Acessível em `http://<IP-DO-ROBO>:8000/calibrate.html`:
- Painel visual com os 12 servomotores organizados por pata (`FL`, `FR`, `HL`, `HR`).
- **Controle Individual de Ângulo**: Sliders interativos para testar cada articulação.
- **Calibração de Pulso (Ticks)**: Ajuste de `tick_min`, `tick_center` e `tick_max` para precisão máxima.
- **Inversão & Trim**: Inverter direção de rotação e aplicar offsets em graus.
- **Ações Rápidas**: Centralizar Todos (90°), Desativar PWM (Detach), Teste de Varredura (Sweep) e Pose em Pé (Stand).
- **Persistência**: Botão "Salvar Calibração" grava as alterações diretamente no `robot/config.json`.

---

## 🔌 Pinout e Conexões de Hardware

### 1. Driver de Servos PCA9685 (I2C)
| Pino PCA9685 | Pino Raspberry Pi | Descrição |
|:---|:---|:---|
| **VCC** | Pino 1 (3.3V) | Alimentação lógica |
| **GND** | Pino 6 (GND) | Terra comum |
| **SDA** | Pino 3 (GPIO 2 - SDA) | Linha de Dados I2C |
| **SCL** | Pino 5 (GPIO 3 - SCL) | Linha de Clock I2C |
| **V+ (Terminal de Borne)** | Bateria Externa (5V-6V / 3A-5A) | Alimentação de força dos servos |

**Mapeamento de Canais do PCA9685:**
- `0`: FL Hip Yaw | `1`: FL Hip Pitch | `2`: FL Knee
- `4`: FR Hip Yaw | `5`: FR Hip Pitch | `6`: FR Knee
- `8`: HL Hip Yaw | `9`: HL Hip Pitch | `10`: HL Knee
- `12`: HR Hip Yaw | `13`: HR Hip Pitch | `14`: HR Knee

### 2. Sensor IMU MPU-6050 (I2C - Endereço `0x68`)
| Pino MPU-6050 | Pino Raspberry Pi | Descrição |
|:---|:---|:---|
| **VCC** | Pino 1 (3.3V) ou Pino 2 (5V) | Alimentação |
| **GND** | Pino 9 (GND) | Terra |
| **SDA** | Pino 3 (GPIO 2 - SDA) | I2C Data (compartilhado com PCA9685) |
| **SCL** | Pino 5 (GPIO 3 - SCL) | I2C Clock (compartilhado com PCA9685) |

### 3. Sensor Ultrassônico HC-SR04 (GPIO)
| Pino HC-SR04 | Pino Raspberry Pi | Observação |
|:---|:---|:---|
| **VCC** | Pino 2 (5V) | Alimentação do sensor |
| **GND** | Pino 14 (GND) | Terra |
| **TRIG** | Pino 16 (GPIO 23) | Sinal de Disparo (Saída do Pi) |
| **ECHO** | Pino 18 (GPIO 24) | **Atenção:** Usar divisor de tensão (ex: 1kΩ / 2kΩ) para converter 5V → 3.3V |

---

## 🚀 Guia de Instalação e Execução

### 1. Configuração do Ambiente Python

Recomenda-se o uso de um ambiente virtual Python:

```bash
# Clonar o repositório
git clone <url-do-repositorio>
cd quadruped_robot

# Criar ambiente virtual
python -m venv venv

# Ativar ambiente virtual
# No Windows:
venv\Scripts\activate
# No Linux / Raspberry Pi:
source venv/bin/activate

# Instalar dependências base
pip install -r requirements.txt
```

---

### 2. Executando em Modo Simulação (PC / Windows / Linux)

No PC, o sistema inicializa automaticamente em modo simulador (Mock):

```bash
# Iniciar o sistema principal
python main.py

# Ou especificando a porta desejada:
python main.py --port 8000
```

Abra o navegador no endereço:
- **Dashboard Principal**: [http://localhost:8000](http://localhost:8000)
- **Estúdio de Calibração**: [http://localhost:8000/calibrate.html](http://localhost:8000/calibrate.html)

---

### 3. Executando no Raspberry Pi (Hardware Real)

1. Habilite a interface I2C no Raspberry Pi:
   ```bash
   sudo raspi-config
   # Vá em: Interface Options -> I2C -> Enable -> Yes
   ```

2. Instale as dependências de hardware no Pi:
   ```bash
   pip install smbus2 RPi.GPIO Adafruit-PCA9685
   ```

3. Verifique se os dispositivos I2C estão visíveis no barramento:
   ```bash
   sudo i2cdetect -y 1
   # Deve exibir 0x40 (PCA9685) e 0x68 (MPU-6050)
   ```

4. Execute o robô:
   ```bash
   python main.py
   ```

5. Acesse o dashboard no seu navegador através do IP do Raspberry Pi na rede local:
   - `http://<IP_DO_RASPBERRY_PI>:8000`

---

### 4. Desenvolvimento do Frontend Web (Opcional)

Para editar o frontend com Live Reload / HMR utilizando o **Vite**:

```bash
cd web
npm install
npm run dev
```

---

## 📡 Protocolo de Comunicação & APIs

### WebSocket (`ws://<HOST>:8000/ws`)
- **Frequência de envio**: ~50 Hz
- **Estrutura de Telemetria (Servidor → Cliente)**:
  ```json
  {
    "type": "telemetry",
    "payload": {
      "timestamp": 12.45,
      "imu": { "roll": 0.5, "pitch": -1.2, "yaw": 3.4 },
      "distance": 45.2,
      "battery": 7.8,
      "joint_angles": [90.0, 45.0, 135.0, ...],
      "cpg_states": { ... },
      "commands": { "gait_enabled": true, "steering": 0.0 },
      "config": { ... }
    }
  }
  ```
- **Envio de Comandos (Cliente → Servidor)**:
  ```json
  { "type": "command", "payload": { "gait_enabled": true, "steering": 0.5 } }
  { "type": "config", "payload": { "cpg": { "a_M": 1.2 }, "feedback": { "K_y": 0.15 } } }
  ```

### Endpoints REST de Calibração
- `GET /api/servos`: Retorna a configuração atual dos 12 servos.
- `POST /api/servo/move`: Move servo específico `{ "index": 0, "angle_deg": 90 }`.
- `POST /api/servo/center-all`: Move todos os servos para 90°.
- `POST /api/servo/detach-all`: Desliga o sinal PWM de todos os servos.
- `POST /api/servo/sweep`: Executa varredura de teste no servo `{ "index": 0 }`.
- `POST /api/servo/stand`: Coloca o robô em postura ereta de teste.
- `POST /api/servo/update-config`: Atualiza calibração de um servo (ticks, trim, inversão).
- `POST /api/servo/save`: Grava a calibração no arquivo `robot/config.json`.

---

## ⚙️ Arquivo de Configuração (`config.json`)

O arquivo [robot/config.json](file:///robot/config.json) armazena todos os parâmetros essenciais:
- **`cpg`**: Parâmetros matemáticos dos osciladores ($\tau_m$, pesos sinápticos $a_M, b_M, c_M, d_M$, acoplamentos $k_{12}, k_{21}, k_M, k_S$).
- **`feedback`**: Ganhos proporcionais de estabilização postural ($K_y, K_p, K_u$).
- **`joints`**: Fatores de escala e offsets gerais das articulações.
- **`servos`**: Calibração individual dos 12 canais do PCA9685 (`channel`, `tick_min`, `tick_center`, `tick_max`, `inverted`, `trim_deg`, limites angulares).

---

## 📄 Licença

Este projeto é desenvolvido para fins educacionais e de pesquisa em robótica quadrúpede e sistemas embarcados.
