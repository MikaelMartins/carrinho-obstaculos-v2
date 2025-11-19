# 🚗🤖 Robô Autônomo Arduino

**Fundamentos Tecnológicos II — Práticas de Extensão**  
**Autor:** Mikael Aurio Martins de Pula da Silva  
**Versão:** 2.4 (Novembro/2025)

Este repositório contém o firmware atualizado do carrinho autônomo baseado em Arduino, utilizando 3 sensores ultrassônicos, servo para varredura e controle individual de 4 motores DC. A versão 2.4 incorpora navegação mais inteligente, desvio lateral imediato, rampa de aceleração e lógica modular.

---

## 📌 Histórico das Versões

### V1.0 — Primeira versão (1 sensor central)
- Apenas um sonar frontal com varredura via servo
- Curvas proporcionais ao ângulo do servo
- Ré curta obrigatória antes de escanear
- Primeira implementação de aceleração gradual

### V2.1 — Inclusão de sensores laterais
- Adiciona 2 sensores ultrassônicos (esquerda e direita)
- Centralização lateral proporcional contínua
- Redução de raspagem em paredes e corredores estreitos
- Correção completa do mapeamento dos 4 motores

### V2.2 — Controle avançado
- Rampa de aceleração suave
- Melhor escolha de trajetória pelo sonar central + servo
- Código modular com funções independentes

### V2.3 — Versão atual
- Ré curta executada apenas quando realmente necessária
- Desvio lateral imediato quando obstáculos aparecem nas laterais
- Varredura inteligente com prioridade real baseada nas leituras
- Melhor organização do loop principal
- Navegação estável, segura e suave

### V2.4 — Preferência Configurável de Direção (atual)
- Adicionada variável global PREFERENCIA_GIRO (0 = esquerda, 1 = direita)
- Sistema de desvio frontal passa a respeitar a preferência antes de decidir pelo servo
- Se ambas as direções forem ruins, o robô recua e escolhe o melhor lado automaticamente
- Organização de comentários e estrutura do código revisada

---

## 🧩 Estrutura Geral do Código

### Motores (AFMotor)
4 motores independentes:

- `motor1`: frente direita  
- `motor2`: frente esquerda  
- `motor3`: traseira esquerda  
- `motor4`: traseira direita

**Funções principais:**
- `setLeftSpeed()` / `setRightSpeed()` → Ajuste fino por lado  
- `runLeft()` / `runRight()` → Direção (FORWARD/BACKWARD)  
- `rampTo()` → Rampa suave até a velocidade máxima  
- `moverFrenteVel()` → Controle de velocidade diferencial  
- `marchaReCurta()` → Ré curta somente quando necessário  
- `virarEsqCurta()` / `virarDirCurta()` → Curvas rápidas

---

## 🎯 Sensores Ultrassônicos (NewPing)

**Configuração:**
- Sonar frontal para detecção principal  
- Sonar esquerdo e direito para distância lateral  
- Leitura com fallback: distâncias "0" são tratadas como 200 cm

**Funções:**
- `medir()` → Medição estável com retardo mínimo  
- `scanComServo()` → Varredura esquerda/centro/direita

---

## 🧭 Lógica de Navegação

**Quando o caminho frontal está livre:**
- Aceleração suave usando `rampTo()`  
- Centralização automática em corredores estreitos  
- Ajuste proporcional com base na diferença entre sensores laterais

**Quando há obstáculo à frente:**
- Pára imediatamente  
- Realiza varredura com o servo  
- Decide direção com base nas maiores distâncias  
- Se não houver direção clara, executa ré + curva aleatória

**Desvio lateral imediato:**
- Se obstáculo estiver muito próximo na esquerda → curva rápida à direita  
- Se obstáculo estiver muito próximo na direita → curva rápida à esquerda

---

## 🔧 Parâmetros Importantes (para calibrar)

| Variável | Função | Valor padrão |
|---|---:|---:|
| LIMITE_FRENTE | Distância mínima frontal | 32 cm |
| RAMP_STEP | Intensidade da aceleração | 6 |
| ANG_LEFT/RIGHT/CENTER | Ângulos do servo | 150/30/90 |
| BASE_SPEED_MIN/MAX | Velocidades base | 120–200 |

**Recomendação:** ajuste o `LIMITE_FRENTE` para o seu ambiente testando objetos diferentes. Coloque o carrinho a 15–25 cm da parede e verifique o ponto ideal onde você deseja que ele comece a desviar.

---

## 📁 Estrutura do Repositório

```cpp
/src
└── codigo_carrinho_v2.3.ino // firmware principal
README.md // documentação
LICENSE // licença do projeto
```

---

## 📜 Licença

Este projeto pode ser distribuído livremente conforme a licença escolhida no repositório.

---

## 🚀 Como Contribuir

- Sugira melhorias na lógica  
- Envie PRs com novas estratégias de desvio  
- Ajuste parâmetros para novos ambientes
