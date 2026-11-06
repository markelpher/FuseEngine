# Fuse Engine — Notas de desenvolvimento

## Objetivo

Registrar o foco de desenvolvimento, as melhorias pendentes e os critérios de validação da engine. A prioridade é evoluir a qualidade visual e o comportamento dos sistemas sem comprometer a estabilidade ou o desempenho.

Para detalhes de arquitetura e integração, consulte [PROJECT.md](PROJECT.md). Os comandos de execução e os controles estão no [README.md](README.md).

## Foco atual: estabilização da aranha

Consolidar uma locomoção consistente e convincente, acompanhada de uma IA previsível e responsiva.

Foram implementadas correções de marcha, perseguição, custo de consultas físicas e gerenciamento de recursos nativos, com cobertura de testes automatizados. A validação visual e de desempenho no jogo completo permanece necessária antes de considerar esse trabalho concluído.

### Validações pendentes no jogo

- [ ] Verificar a naturalidade dos passos, das curvas e das paradas.
- [ ] Confirmar a perseguição pelo `F3` e a pausa e retomada da IA pelo `F6`.
- [ ] Avaliar desvios de obstáculos e transições entre superfícies em mapas reais.
- [ ] Medir o custo da aranha na CPU e o impacto no tempo de frame, incluindo situações de bloqueio.
- [ ] Executar sessões prolongadas para verificar estabilidade e uso de memória.

## Melhorias pendentes

| Sistema | Trabalho previsto |
| --- | --- |
| Oceano | Investigar o custo de renderização e revisar a aparência da superfície, preservando a coerência com a física da água. |
| Terreno | Medir e otimizar o custo do material utilizado no mapa `TT.bth`, mantendo a qualidade visual necessária. |
| Fog volumétrico | Reproduzir e corrigir os artefatos visuais relatados. |

A ordem de execução desses itens deve ser definida a partir da gravidade dos problemas e das medições obtidas, não apenas da complexidade aparente de cada sistema.

## Critérios de trabalho e validação

- Priorizar estabilidade e correção antes de adicionar novos recursos.
- Reproduzir o problema antes de alterar o sistema responsável.
- Comparar desempenho com o mesmo mapa, câmera, resolução e configuração.
- Registrar tempo de frame e custos de CPU/GPU; não avaliar desempenho apenas pelo FPS.
- Preservar os limites de custo e as verificações de segurança dos sistemas físicos.
- Executar os testes relacionados e validar o resultado visualmente no jogo ou no editor.
- Manter alterações focadas e documentar limitações conhecidas e próximos passos.

## Registro de problemas

Ao investigar um problema, registrar:

1. **Contexto:** mapa, posição da câmera, hardware, configuração e versão executada.
2. **Reprodução:** sequência mínima de ações que produz o comportamento.
3. **Resultado esperado e observado:** descrição objetiva, com captura ou log quando disponível.
4. **Medições:** tempo de frame, seções relevantes do profiler e uso de memória.
5. **Validação:** testes executados, resultado após a correção e limitações restantes.
