# CLAUDE.md — XBbot FRC 2025

## Missão e contexto

Desenvolver o software do robô da equipe XBbot para a temporada **2025 REEFSCAPE**, utilizando **Java e WPILib**. Este documento registra o contexto inicial e deve evoluir conforme as decisões forem confirmadas pelo responsável pelo projeto.

## Regra prioritária: não antecipar decisões

O projeto está em fase de planejamento. **Não implemente código de robô, subsistemas, comandos, máquinas de estado ou infraestrutura até que o responsável solicite e os requisitos relevantes estejam definidos.**

Antes de propor uma arquitetura ou alteração estrutural:
1. Leia este documento e o restante da documentação relevante.
2. Considere os objetivos e restrições já definidos.
3. Identifique informações ausentes ou ambiguidades.
4. Pergunte quando uma decisão necessária ainda não tiver sido tomada; não preencha lacunas com suposições.

Nunca invente:
- IDs de CAN, PWM, DIO, canais ou endereços de hardware;
- fabricantes/modelos de motores, controladores, sensores ou câmeras;
- dimensões, reduções, offsets, limites ou unidades mecânicas;
- setpoints, tolerâncias, velocidades ou ganhos de controle;
- nomes e responsabilidades definitivas de estados, comandos e subsistemas;
- requisitos de visão, localização ou estratégia de pose estimation.

## Mecanismos conhecidos

1. **Rampa passiva de coral:** mecanismo articulado em direção à estação de coral. A rampa foi descrita como passiva; não presuma que tenha motor ou atuador próprio.
2. **Manipulador de coral:** mecanismo articulado com rodas motorizadas.
3. **Elevador:** sistema de elevação.
4. **Intake de algae:** mecanismo articulado com rodas motorizadas.

A interação entre mecanismos, sensores, atuadores, limites e sequências operacionais ainda precisa ser detalhada.

## Objetivos técnicos

- Controle coordenado da superestrutura com abordagem baseada em **máquinas de estado**.
- Subsistemas com responsabilidades claras.
- Camada de **IOs** que mantenha a lógica de controle desacoplada do hardware.
- Logging e telemetria úteis para depuração.
- Suporte a **replay determinístico**, integrado à solução de logging escolhida.
- Desenvolvimento do **Vision Subsystem**, que ainda está faltando e será especificado posteriormente.

## Referências arquiteturais

### biocore2026sim v2

A camada de IOs existente em `biocore2026sim v2` é uma referência importante e deve ser preservada/adaptada como ponto de partida arquitetural. Não descarte essa abstração sem uma justificativa técnica discutida com o responsável pelo projeto.

### Código da equipe 2910 FRC

O código da equipe 2910 é uma referência para estudar organização e decisões arquiteturais. Não copie sua estrutura cegamente. Avalie cada ideia em relação ao robô, aos requisitos e à temporada **2025 REEFSCAPE**.

## Diretrizes para agentes

- Não misture lógica de alto nível com chamadas diretas específicas de hardware quando a camada de IOs for apropriada.
- Mantenha entradas e saídas de IO explícitas e testáveis.
- Não adicione dependências, bibliotecas ou ferramentas sem verificar compatibilidade com a versão de WPILib/Gradle do projeto.
- Não altere versões de ferramentas ou estrutura do projeto sem necessidade e sem comunicar o impacto.
- Prefira mudanças pequenas, coerentes e fáceis de revisar.
- Não afirme que algo foi testado se os testes/build não foram executados.
- Ao concluir uma tarefa, informe arquivos alterados, decisões tomadas, dúvidas restantes e verificações executadas.

## Estado inicial

Este repositório contém apenas documentação de contexto. A arquitetura detalhada e a implementação serão definidas em etapas, depois que o responsável terminar de apresentar os objetivos e requisitos.
