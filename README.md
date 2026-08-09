# RebuiltHyobots 2026

Código do robô da equipe para a temporada **REBUILT 2026** (FRC), em Java com WPILib 2026.

Este é o codebase canônico da equipe. Ele nasceu da fusão das melhores ideias de vários outros
repositórios de FRC (1156, 1678, 254, 2910, 5000, 5940, 6328, 9199), adaptadas às convenções daqui —
não copiadas cruas. Cada arquivo trazido de fora traz um comentário no topo dizendo de onde veio.

---

## Como compilar e rodar

```bash
./gradlew build          # compila, formata (spotless) e roda os testes
./gradlew test           # só os testes
./gradlew simulateJava   # abre o simulador
./gradlew deploy         # envia para o roboRIO
```

> O [`gradle.properties`](gradle.properties) fixa o JDK do WPILib. Sem isso o Gradle pega a JVM que
> o lançou (numa das máquinas da equipe, a JBR do Android Studio), e aí o spotless quebra e qualquer
> teste que toque no HAL derruba a JVM inteira. Se você mover a instalação do WPILib, ajuste o
> caminho ali.

### CI

[`.github/workflows/build.yml`](.github/workflows/build.yml) roda `spotlessCheck` e `build` a cada
push na `main` e em todo pull request. Se um teste quebrar, o relatório fica disponível como artefato
do job.

O runner não tem o JDK do WPILib, então cada chamada do Gradle sobrescreve o `org.gradle.java.home`
pela linha de comando (`-D` tem precedência sobre o `gradle.properties`) apontando para o Temurin 17
que o `setup-java` instalou. Se você mudar a versão de Java do projeto, mude nos dois lugares.

O `spotlessCheck` roda **antes** do `build` de propósito: como o `compileJava` depende do
`spotlessApply`, o build formata sozinho — a checagem tem que vir primeiro pra de fato reprovar
código desformatado em vez de arrumá-lo em silêncio.

---

## Arquitetura

### Duas raízes de pacote

| Pacote | O que vive nele |
|---|---|
| `frc.frc_java9485` | Biblioteca: classes-base, utils, constantes, wrappers de motor, loggers, joysticks |
| `frc.robot` | O robô em si: subsistemas, comandos, `Robot`/`RobotContainer` |

A regra prática: se serviria para o robô do ano que vem, vai em `frc_java9485`.

### Camada de IO (AdvantageKit)

Todo mecanismo tem uma interface `XxxIO` com inputs anotados `@AutoLog`, e pelo menos duas
implementações: `XxxIOTalonFX` (hardware) e `XxxIOSim` (simulação). O `RobotContainer` escolhe qual
injetar. Isso é o que permite rodar a lógica inteira no simulador e reproduzir logs depois.

### Classes-base de mecanismo — `frc.frc_java9485.bases`

Todo mecanismo segue o padrão *wanted state → system state*, e o boilerplate disso mora em três
classes (adaptadas do `frc.lib.bases` do 1678 e do `ServoMotorSubsystem` do 254):

- **`StateMachineMechanism<W, S, In>`** — lê os inputs, publica, roda a transição, aplica o estado,
  loga o par de estados e cronometra o loop. Um mecanismo concreto só implementa três métodos.
- **`ServoMechanism`** — controle de posição: setpoint com clamp no curso mecânico, `atSetpoint()`
  por epsilon, auto-homing na borda de subida de um sensor.
- **`FlywheelMechanism`** — controle de velocidade: setpoint em RPM, `spunUp()` com tolerância e
  debounce avaliado uma vez por loop.

Esses mecanismos **não** são `Subsystem` do WPILib de propósito: eles são bombeados pelo dono
(`SuperStructure` / `ShooterSubsystem`), então nenhum comando consegue requisitá-los diretamente e
brigar pelo controle.

- **`StateGraph<S>`** (do 254) — os estados do `SuperStructure` são nós de um grafo, e movimentos
  legais são arestas com custo. Um pedido de transição ilegal é roteado pelo caminho mais curto até
  um estado seguro em vez de saltar direto. É o que impede, por exemplo, ir de SHOOTING direto para
  COLLECTING baixando o intake com o flywheel ainda cheio.

### `RobotState`

Snapshot único e compartilhado do robô: pose fundida, velocidades field-relative, atitude do giro
(pitch/roll), última estimativa de visão aceita e mensagem do FMS. Alimentado pelo swerve e pela
visão a cada loop, lido por comandos e mecanismos — assim ninguém precisa alcançar dentro de outro
subsistema.

### Simulação — `frc.frc_java9485.sim`

Os `*IOSim` são simulações físicas de verdade, não loopback. Três classes-base (adaptadas do
`frc.lib.sim` do 1678) embrulham as sims da WPILib:

| Classe | Base WPILib | Para |
|---|---|---|
| `PivotSim` | `SingleJointedArmSim` | Turret, hood, pivô do intake — inércia, gravidade e batentes |
| `RollerSim` | `FlywheelSim` | Flywheel, index, rolos do intake — curva de aceleração real |
| `LinearSim` | `ElevatorSim` | Climber, conveyor — massa e gravidade |

Isso importa porque o loopback antigo andava 20% em direção ao setpoint por loop: suave, instantâneo
e monotônico — nada que um mecanismo real seja. Ele escondia exatamente o que vale testar. Com a
física, o flywheel leva tempo pra subir, o hood cede sob gravidade, o turret bate no batente — e aí o
debounce de spun-up e o `ShotVerifier` passam a ser exercitados no simulador.

As propriedades físicas ficam todas em [`SimConsts`](src/main/java/frc/frc_java9485/constants/sim/SimConsts.java),
isoladas de propósito: **nenhum código de controle lê esse arquivo**, então um valor errado ali
muda como o mecanismo simulado acelera e nunca o que o robô real faz. Os momentos de inércia são os
que mais importam — são eles que definem o tempo de acomodação.

> Testes que exercitam as sims precisam de `SimHooks.pauseTiming()` + `stepTiming(0.02)`, porque as
> sims integram contra o relógio do FPGA. Atenção: o `Debouncer` da WPILib usa o `MathSharedStore`,
> que fora de um robô rodando cai pra tempo de relógio de parede — então debounce não avança sob
> relógio pausado. No robô os dois andam juntos.

### `VirtualSubsystem`

Lógica periódica que não é um `Subsystem` (calculadora de tiro, visualizadores) estende
`VirtualSubsystem` e é bombeada pelo `Robot.robotPeriodic()`, antes e depois do scheduler.

---

## O shooter

Este é o núcleo do robô, então vale detalhar.

```
RobotState (pose + velocidade)
        │
        ▼
  ShotCalculator ──► ShotSolution { heading, turret, hood, RPM, distância, tempo de voo }
        │                    │
        │                    ├──► TurretSubsystem  (ângulo)
        │                    ├──► HoodSubsystem    (posição)
        │                    ├──► FlyWheelSubsystem(RPM)
        │                    ├──► AimRobotToHub    (heading do chassi)
        │                    └──► ShotVisualizer   (arco previsto)
        ▼
  ShotVerifier ──► pode atirar? (mirado, parado, nivelado, no alcance)
```

**`ShotCalculator`** é a única fonte de verdade da solução de tiro. Ele resolve iterativamente o
tempo de voo e move o ponto de mira para um "alvo virtual" que compensa a velocidade do robô — assim
dá para atirar em movimento. A iteração de ponto fixo vem do `NewtonShotCalculator` do 5940; a
suavização das saídas, do `LaunchCalculator` do 6328.

**`ShotVerifier`** (do 1678) responde a pergunta que "os setpoints foram atingidos" não responde:
*vale a pena atirar?* Ele rejeita o tiro se o chassi não está mirado, se ainda está girando rápido
demais (varrer o ângulo certo não é o mesmo que estar parado nele), se o robô está tombado, ou se a
distância está fora da faixa calibrada. O `SuperStructure` só libera o index/conveyor quando os
mecanismos estão no setpoint **e** o tiro verifica.

### ⚠️ O shot model ainda não está calibrado

As tabelas em `TurretConsts.ShotModel` (`DISTANCE_TO_RPM`, `DISTANCE_TO_HOOD_POSITION`,
`DISTANCE_TO_TOF`) são **valores inventados**. Todo o resto do shooter é tão bom quanto elas.

Para calibrar, use o **`CharacterizeShotCommand`**:

1. Estacione o robô a uma distância do hub.
2. Ajuste `ShotCharacterization/HoodPosition` e `ShotCharacterization/FlywheelRPM` no dashboard até
   os tiros entrarem.
3. Chame `recordSample()` (ligue num botão) para gravar aquele ponto.
4. Repita em 5–8 distâncias ao longo da faixa útil.
5. Ao encerrar o comando, ele imprime cada amostra já no formato `DISTANCE_TO_RPM.put(...)` pronto
   para colar, mais um ajuste de reta e o R² de cada coluna — um R² baixo é o sinal de que a relação
   não é linear ali (ou de que tem amostra ruim).

Opcionalmente, `markShotReleased()` / `markShotLanded()` cronometram o tempo de voo real e alimentam
o `DISTANCE_TO_TOF`.

---

## Autônomo

Os autos são arquivos `.auto` do PathPlanner em `src/main/deploy/pathplanner/autos`. O pacote
[`frc.frc_java9485.autonomous`](src/main/java/frc/frc_java9485/autonomous/) cuida do resto:

| Classe | Papel |
|---|---|
| `AutoChooser` | Lista os `.auto` do deploy no dashboard. Falha de leitura vira erro + `Alert`, não um chooser vazio inexplicável |
| `AutoManager` | Pré-constrói o auto selecionado, valida a pose inicial e loga a execução |
| `AutoCommands` | As ações que os marcadores executam |
| `RegisterNamedCommands` | Só a tabela de nomes → ações |

**O auto é construído com o robô desabilitado**, no instante em que a seleção muda. Construir um
`PathPlannerAuto` lê o arquivo, parseia as trajetórias e monta a árvore de comandos — fazer isso
dentro do `autonomousInit()` gasta dezenas de milissegundos no pior momento da partida.

**A pose inicial é conferida enquanto o robô está desabilitado.** O `AutoManager` compara a pose atual
com o início do auto selecionado (já com a flip de aliança aplicada pelo utilitário do próprio
PathPlanner, pra não discordar dele) e levanta um `Alert` se o robô estiver fora do lugar. Robô mal
posicionado é a falha de autônomo mais comum, e ela é detectável antes da partida.

### Marcadores disponíveis

| Marcador | O que faz |
|---|---|
| `AimAndShoot` | Gira pro hub **e** acelera o flywheel em paralelo, atira quando o tiro verifica |
| `Shoot` | Acelera e atira quando verifica (assume que o caminho já mirou) |
| `WaitForShotReady` | Segura o caminho até o tiro verificar |
| `AimAtHub` | Só gira pro hub |
| `Collect` / `Pass` / `Eject` / `Climb` | As ações do superstructure |

O ponto do `AimAndShoot`: **a janela de alimentação começa depois que o tiro verifica**, não quando o
marcador dispara. Um `shoot().withTimeout(2.0)` conta desde o início — se o flywheel leva 1,5 s pra
acelerar, o robô só alimenta por 0,5 s. Aqui ele espera o `ShotVerifier` confirmar (mirado, parado,
nivelado, no alcance) e só então alimenta pela janela inteira, com o timeout como rede de segurança.
Esse comportamento é o que o `AutoCommandsTest` verifica.

> Os marcadores em português (`coleta`, `shootar`, `Climbar`...) são os que os `.auto` atuais já
> referenciam — removê-los quebraria todos os autos existentes em silêncio. Use os nomes em inglês em
> caminhos novos; quando os antigos forem re-salvos no PathPlanner, o bloco legado pode sair.

## Comandos

| Comando | O que faz |
|---|---|
| `ShootOnTheMove` | Atira sem parar: o piloto mantém translação, o chassi persegue a solução compensada e o tiro sai quando verifica |
| `ShiftAwareShooting` | Roda o superstructure pelo cronograma de turnos do hub — pré-acelera antes da janela abrir |
| `KeepTurretInRange` | Gira o chassi **só** quando o turret está sem curso (`TurretChassisAllocator`) |
| `DriveIntoShotRange` | Vai ao ponto mais próximo de onde o tiro é válido |
| `ShotCorrectionPolicy` | Corrige o chassi conforme o *motivo* da rejeição do tiro |
| `GatedClimb` | Climber só com endgame + zona certa + mecanismo recolhido |
| `AntiTipDrive` | Limite de aceleração que aperta quando o CG sobe ou o robô inclina |
| `PitDiagnostics` | Varre cada mecanismo e reporta quais não se moveram |

Três valem detalhar:

**`ShootOnTheMove`** — o `ShotCalculator` já resolvia o alvo virtual compensando a velocidade do
chassi, e nada usava isso enquanto o robô transladava. O detalhe que faz funcionar é o **feedforward
na velocidade do alvo**: enquanto o robô anda, a direção do alvo virtual se move sozinha, e um P puro
no erro de heading fica sempre atrasado — com o atraso crescendo justo quando o robô acelera. Então a
taxa de variação do alvo é medida (diferença finita com wrap, filtrada) e somada ao omega.

**`ShiftAwareShooting`** — o hub só pontua pra sua aliança em janelas de 25 s, e quais janelas depende
de quem ganhou o autônomo. Um flywheel que começa a acelerar quando a janela abre desperdiça os
primeiros segundos dela. Este comando usa o estado `PREPARING` (mira e acelera, **não** alimenta) pra
já estar pronto em t=0.

**`TurretChassisAllocator`** — o turret tem ~+96°/−106° de curso, e pedir além disso hoje apenas
clampa em silêncio. Este divide o heading exigido entre turret e chassi, com **banda de histerese**:
sem ela o chassi entra, alivia o turret, sai, sobrecarrega o turret de novo — e o robô oscila parado.

> `ShootOnTheMove` e `DriveIntoShotRange` dependem do shot model calibrado. Atirar em movimento com
> tabelas inventadas erra com mais confiança, não menos.

## Unidades — leia isto antes de mexer

Unidade errada é a classe de bug mais cara aqui. As convenções atuais:

| Grandeza | Unidade | Onde converte |
|---|---|---|
| Ângulo do turret | **graus** em toda a camada de mecanismo | `TurretIOSparkMax` converte para rotações do motor |
| Posição do hood | **unidades de mecanismo** (rotações do motor), não graus | — |
| Velocidade do flywheel | **RPM** no motor | — |
| Distâncias do campo | **metros** | — |

Os Sparks rodam sem *position conversion factor*, então `getPosition()` devolve rotações do motor.
A conversão graus ↔ rotações vive só em `TurretConsts.Setpoint` e é testada em `TurretUnitsTest`.

---

## Swerve

Configurado por JSON do YAGSL em [`src/main/deploy/swerve`](src/main/deploy/swerve). Os módulos usam
**Kraken X60** (`krakenx60`) para drive e angle, com CANcoder absoluto e Pigeon2.

| Dispositivo | Tipo | IDs |
|---|---|---|
| Drive / Angle | TalonFX (CTRE) | 1–8 |
| Encoders absolutos | CANcoder (CTRE) | 10–13 |
| IMU | Pigeon2 (CTRE) | 9 |

> ⚠️ **Os ganhos de PID no [`pidfproperties.json`](src/main/deploy/swerve/modules/pidfproperties.json)
> são pontos de partida, não valores medidos.** Eles precisam ser ajustados com o robô no cavalete
> antes de dirigir. Ver a seção abaixo para o porquê da escala.

### Por que os ganhos mudaram de escala ao trocar de SparkMax para TalonFX

Não é a mesma unidade. O YAGSL controla TalonFX com `MotionMagicVoltage` (angle) e `VelocityVoltage`
(drive), e configura `Feedback.SensorToMechanismRatio` — então o erro está em **rotações do mecanismo**
e a saída em **volts**. No SparkMax os ganhos eram fração de duty cycle sobre as unidades convertidas
da REV.

Na prática: o kP de angle era `0.005`. Com TalonFX isso significa que um erro de meia rotação (180°)
produziria 0.0025 V — o módulo simplesmente não giraria. Por isso o valor subiu para a ordem de
dezenas. O campo `f` do JSON vira `kS` (volts de atrito estático), não um feedforward de velocidade.

### O que ainda precisa ser conferido no robô

- **Sentido de rotação.** A convenção de inversão do CTRE não é a mesma da REV. Se um módulo girar
  para longe do setpoint em vez de para ele, inverta `inverted.angle` naquele módulo.
- **Limites de corrente.** Continuam em 40 A (drive) / 20 A (angle), que eram dimensionados para NEO.
  Um Kraken/Falcon aguenta bem mais; subir o limite de drive é ganho de aceleração disponível.
- **`DriveConsts.MAX_SPEED`** está em 4.0 m/s, calculado para o NEO com a redução 6.75 e roda de
  3.85". Kraken X60 daria ~4.55 m/s e Falcon 500 ~4.84 m/s — o valor atual é válido, só conservador.
- **Variante FOC.** Está como `krakenx60`. Se vocês tiverem licença Phoenix Pro, `krakenx60foc` dá
  mais torque e um modelo de motor mais preciso.

## Motores

**Todo motor do robô é um Kraken X60 em um TalonFX.** O mapa completo do barramento está em
[`CanIds`](src/main/java/frc/frc_java9485/constants/robot/CanIds.java).

- **CTRE Phoenix 6** (`frc.frc_java9485.motors.ctre`) — `TalonFXMotor` mais o pacote `phoenix6`
  (`Phoenix6Util`, `TalonFXConfigEquality`, `TalonFXFactory`, `CanDeviceId`, `StatusSignalRefresher`).
  Os mecanismos usam isto pelos `*IOTalonFX`; o swerve usa a camada própria do YAGSL.
- **REV** (`frc.frc_java9485.motors.rev`) — `SparkMaxMotor`, `SparkFlexMotor`, `SparkMaxBrushed`
  continuam na biblioteca, mas **nenhum mecanismo usa**. Ficam disponíveis caso algum subsistema
  volte a ser REV; as classes-base são agnósticas de fabricante, então é só trocar o IO injetado.

O destaque do lado CTRE é o `Phoenix6Util.applyAndCheckConfiguration()`: ele aplica a config, **lê de
volta do motor**, compara campo a campo e repete se não bateu. "Chamei apply()" e "o motor está
configurado" não são a mesma afirmação.

### Unidades por mecanismo depois da migração

O Phoenix aplica a redução no próprio dispositivo (`SensorToMechanismRatio`), então cada mecanismo
escolheu o que faz mais sentido:

| Mecanismo | `SensorToMechanismRatio` | Posição significa |
|---|---|---|
| Turret | redução (66.8) | **rotações do turret** — o IO só divide por 360 para virar graus |
| Hood | 1 | rotações do motor (preserva a faixa 0–3.5 e a calibração do shot model) |
| Climber | 1 | rotações do motor (mesma caixa de antes, limites preservados) |
| Flywheel | 1 | direto; velocidade em rot/s convertida para RPM no IO |

O turret é o que mais ganhou: a conversão graus↔rotações-do-motor feita à mão — onde vivia o bug de
unidade — virou uma divisão por 360.

> ⚠️ **Os ganhos de PID de todos os mecanismos foram reescalados** de duty cycle (REV) para **volts**
> (Phoenix 6), com o erro em unidades de mecanismo. São pontos de partida, não medições. Precisam ser
> ajustados no cavalete.

> ⚠️ **O intake precisa ser recalibrado.** O pivô era lido por um encoder through-bore alternativo no
> SPARK MAX; o Kraken não tem essa entrada e agora mede no rotor. Os setpoints em
> `IntakeConsts.Setpoint` (250 / 200 / 16) não descrevem mais os mesmos ângulos físicos.

---

## Logging

Tudo passa pelo AdvantageKit. Chaves úteis no AdvantageScope:

| Chave | O que mostra |
|---|---|
| `Subsystems/<Mecanismo>/` | Inputs, wanted state, system state, setpoint, medido |
| `ShotCalc/` | Solução de tiro completa + alvo virtual |
| `ShotVisualizer/Arc` | Arco previsto do tiro, em 3D |
| `Visualizer/Components` | Pose 3D de cada parte móvel (turret, hood, intake, climber) |
| `LoggedTracer/` | Tempo de cada fase do loop, total (`LoopMS`), fase mais lenta (`WorstEpoch`) e nº de estouros |
| `VirtualPD/` | Corrente e energia por mecanismo — para diagnosticar brownout depois do jogo |
| `Zones/` | Contornos das zonas do campo |

`LoggedTunableNumber` expõe qualquer constante no dashboard sob `/Tuning` durante o ajuste.

### Alertas

`RobotAlerts` publica `Alert`s da WPILib que aparecem sozinhos no Elastic/AdvantageScope: bateria
baixa, uso de CAN alto, erros de recepção, brownout — e **estouro de loop**.

O alerta de loop lê o que o `LoggedTracer` já mede e diz *qual fase* estourou o orçamento de 20 ms.
Um robô rodando a 15 Hz em vez de 50 Hz parece "mole" e leva a culpa pro PID; esse alerta aponta o
culpado real, que costuma ser visão, logging ou um mecanismo fazendo trabalho a mais. Ele só acende
se houve estouro nos últimos 2 segundos, pra que uma pausa de GC isolada não vire ruído permanente.

### Logs comprimidos (`.wpilogxz`)

[`frc.frc_java9485.utils.logger.wpilogxz`](src/main/java/frc/frc_java9485/utils/logger/wpilogxz/)
escreve o WPILOG através de um stream LZMA2. O formato dos registros é idêntico — só os bytes em
disco são comprimidos — então um `.wpilogxz` descomprime num `.wpilog` byte a byte igual.

Vale porque log de robô é dominado por valores que quase não mudam entre loops, o que comprime muito.
É isso que decide quantas partidas cabem no armazenamento do roboRIO antes de alguém ter que apagar
justo o log que precisava.

| Classe | Papel |
|---|---|
| `WPILOGXZWriter` | `LogDataReceiver` do AdvantageKit — substituto direto do `WPILOGWriter` |
| `WPILOGXZReader` | `LogReplaySource` — replay de um log comprimido |
| `WPILOGXZEncoder` / `WPILOGXZDecoder` | O codec em si |
| `WPILOGExtractor` | CLI que extrai campos numéricos pra CSV (comprimido ou não) |

**Está desligado por padrão** (`RobotConsts.USE_COMPRESSED_LOGS`). Ligue depois de confirmar neste
robô que o orçamento de tempo de loop absorve a compressão e que a sua build do AdvantageScope abre
o formato. O custo é CPU a cada flush.

Extrair campos pra análise fora do AdvantageScope — útil pra conferir o shot model contra o que
aconteceu de fato na partida:

```
./gradlew run -PmainClass=frc.frc_java9485.utils.logger.wpilogxz.WPILOGExtractor \
    --args="logs/akit_26-03-14_10-22-01_q12.wpilogxz ShotCalc/CompensatedDistanceM ShotCalc/FlywheelRPM"
```

> Isso adiciona a dependência `org.tukaani:xz:1.9` ao [build.gradle](build.gradle).

---

## Pendências conhecidas

Estas estão marcadas com `TODO` no código e valem ser resolvidas antes da competição:

- **Shot model não calibrado** — veja a seção do shooter acima. É o item mais importante.
- **`VisionConsts`** — o transform robô→câmera do PhotonVision está zerado; precisa ser medido no
  robô real (por isso `RASPBERRY_ENABLED` está desligado).
- **`ClimberConsts`** — CAN ID real e curso (soft limits) ainda não definidos.
- **`TurretConsts.PID`** — ganhos são um ponto de partida conservador, não medidos.
- **`VisualizerConsts`** — as geometrias dos componentes 3D são aproximadas; só afetam o desenho.
- **`SimConsts`** — inércias, reduções e massas são estimativas; só afetam o simulador. Meça (ou tire
  do CAD) quando o robô estiver montado, começando pelos momentos de inércia.
- **Convenção do `AllianceFlip`** — assume campo **espelhado** (X inverte, Y mantém). Se o campo
  REBUILT for rotacionalmente simétrico, todos os métodos precisam mudar juntos.
- **Arquivos mortos** — `src/test/RobotContainerTest.java` e `src/test/teste.java` estão fora de
  `src/test/java`, nunca compilam e referenciam APIs que já não existem.
