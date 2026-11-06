# Spider AI headless integration gate

Harness isolado que referencia `Fuse` e executa **SpiderEnemy + SpiderPatrol/planner reais + gait do FBX real + Jolt**, sem janela, contexto OpenGL, `AssetManager`, renderização ou áudio. Não substitui decisões do patrol/planner e não injeta direções/velocidades no motor.

**Validação atual Release: 14/14 PASS**, incluindo regressões de latch após ClearTarget e de chegada falsa num piso 1,2 m acima. Padrão `resume` confirmado sem informar `--f3-override`. As assertions anteriores permanecem intactas. Resultados atuais em `results-review-edges/`; `results-final/` preserva a validação anterior de 12 casos, incluindo duas repetições extras de hysteresis.

## Executar

Na raiz do repositório (SDK .NET 10/Windows e dependências nativas do Fuse):

```sh
dotnet build tests/SpiderAi.Integration/SpiderAi.Integration.csproj -c Release --nologo -v minimal
dotnet run --project tests/SpiderAi.Integration/SpiderAi.Integration.csproj -c Release --no-build -- --case all --output tests/SpiderAi.Integration/results-review-edges
```

O primeiro build e a primeira execução foram invocados separadamente, **cada um com timeout externo de 120 s**. Não há processo/watch persistente. Use o mesmo limite no runner/CI. Exit codes: `0` sem assertions falhando, `1` assertions/erros de cenário, `2` erro na preparação global. Uma falha não impede a execução dos cenários seguintes.

`--case` aceita `all`, `stationary`, `moving`, `airborne`, `near-arrival`, `f3-off`, `f6-off-f3-on`, `low-box`, `blocked`, `hysteresis`, `target-lost-reacquire`, `target-lost-reacquire-boundary`, `detour-retarget`, `arrived-clear-near`, `elevated-near-state`.

Outras opções:

- `--output PATH`: CSV por cenário, `summary.json`, `environment.json`, `sources-changed-during-run.json`.
- `--max-update-ms 250`: teto de latência de um update (padrão 250 ms).
- `--max-p95-ms 16.666667`: teto de p95 do update (padrão um frame de 60 Hz). É configurável para máquinas lentas; o baseline abaixo usa o padrão.
- `--f3-override observe|resume|stay-disabled`: padrão obrigatório do gate **resume**, inclusive quando a flag é omitida. O contrato escolhido exige `SetPursuitEnabled(true)` reativando `Enabled` imediatamente e >=3 m de progresso depois de F3. O harness não reativa `Enabled` manualmente para produzir esse resultado. `observe` e `stay-disabled` são opções legadas de diagnóstico, não o gate usado nos relatórios atuais; `stay-disabled` deve falhar com o contrato atual.

Para executar explicitamente o contrato F3/F6 (também é o padrão):

```sh
dotnet run --project tests/SpiderAi.Integration/SpiderAi.Integration.csproj -c Release --no-build -- --case f6-off-f3-on --f3-override resume --output tests/SpiderAi.Integration/results-f3-override
```

`bin/`, `obj/` e `results*/` são ignorados pelo `.gitignore` **local deste novo projeto**. Não há alteração na solution, em produção ou nos outros testes.

## Wiring e medição

- `SkinnedModelLoader.LoadRig(Fuse/res/skinned_models/spider.fbx)` carrega somente CPU; skeleton independente por cenário.
- `new SpiderEnemy(id, assets: null)`, sua própria `Body` construída como cápsula kinematic com o profile real, `Entity` de `scene.ActiveScene.Add(null, id, body)` atribuída pelo setter privado via reflection. Não chama `SpiderEnemy.Initialize`/`Update`, que inicializam assets/hitboxes/áudio.
- Somente o mínimo do helper `LegData`/`ProceduralSpiderWalk.Initialize` do harness de locomotion foi copiado, com os oito conjuntos de nós do FBX; não há pés sintéticos.
- Solver compartilhado por patrol/planner, motor e gait; corpo excluído de superfícies caminháveis, `BodyPoseValidator = gait.CanOccupyPose`, descrição de rejeição e `PoseReset -> gait.ResetContacts` reais. Campos privados de locomotion de `SpiderEnemy` apontam para essas mesmas instâncias.
- **Todo frame**, inclusive os 30 frames de settling: `Patrol.Update(1/60) -> gait.Update(1/60, SpiderLocomotionPose real) -> physics.Step(1/60)`.
- O settling usa os flags desabilitados; a fase medida usa `SetPursuitEnabled`/`SetPursuitTarget`/`SetPursuitTargetSurface` públicos. `airborne` chama `ClearPursuitTargetSurface` a cada frame.
- `PatrolMs` inclui planner + motor; `GaitMs` inclui gait/IK. `UpdateMs` é a soma das duas janelas, **não inclui** aquisição da pose, `physics.Step`, reflection de diagnóstico, assertions e CSV. Os query counters são lidos depois do gait, sem reset extra do solver: orçamento compartilhado do update inteiro.
- Cleanup remove native bodies antes de liberar suas shapes, motor e solver são descartados, flags/alvo/superfície estáticos são restaurados. Preserva **HasPursuitTarget**, chamando `ClearPursuitTarget()` antes de restaurar somente um alvo que existia (não usa `SetPursuitTarget(Vector3.Zero)` para representar ausência). Restaura `Enabled` **depois** de `SetPursuitEnabled`, para respeitar o novo efeito de reativação. Verifica a restauração de flags/presença/superfície. Não chama teardown de enemy inicializado com assets nulos.
- CSV registra `HasPursuitTarget`, estado público `SpiderAiState` e fase do cenário; summary inclui contagens/transições de estado, eventos de lifecycle e suas métricas. `NavigationTarget` público substitui reflection para consultar o destino do patrol.
- `environment.json` registra hashes do FBX, assemblies, fontes presentes no início da execução, MVID, runtime e limites. A lista de fontes alteradas durante a execução distingue resultados do assembly carregado de edições concorrentes. Os hashes de fontes no início não são uma garantia de snapshot atômico durante o build.

## Cenários e assertions (não relaxadas após o baseline)

Chão: caixa estática cobrindo X/Z ±100 m, superfície Y=0. Spawn `(0, BodyClearance, 0)`, frente +Z. Profile padrão: clearance 0,83 m, velocidade máxima 9,5 m/s, tolerância de contato 0,015 m.

| Cenário | Estímulo e requisito |
|---|---|
| `stationary` | Alvo fixo `(0,0,18)` com superfície +Y/body do chão; 900 frames. Aproximar e parar. |
| `moving` | Alvo parte de Z=18, move +Z a 0,8 m/s por 8 s e para em Z=24,4; 1080 frames. Avanço >=20 m, distância média <=8 m em t=6..8 s, depois chegada/parada. |
| `airborne` | Alvo **raw** `(0,4,18)`, sem superfície; 900 frames. Perseguir a projeção tangencial no chão e parar sem voar em direção à altura do alvo. Não fabrica uma superfície para o player. |
| `near-arrival` | Alvo a apenas 1 m, já dentro do raio de chegada de 1,25 m; 360 frames. Parar sem oscilar. |
| `f3-off` | Persegue por 180 frames; desativa F3 e continua mudando o alvo raw/superfície para longe, em outra direção; mais 420 frames. Zero plano de pursuit após OFF, nenhum target de patrol copiando o player, >=1 m de caminho e >=30 frames de patrol Walking com distância restante >0,8 m e velocidade >0,05 m/s. Min/max wait são ambos 0,1 s para não confundir espera com falha de retomada; as direções aleatórias de patrol continuam reais. |
| `f6-off-f3-on` | 600 frames: `Enabled=false` inicialmente; F3 ON no frame 120 deve reativar AI. No frame 240 pausa novamente via `Enabled=false`, mantendo F3 ON; no frame 300 retoma somente via `Enabled=true`. Exige `Disabled` + intenção zero enquanto pausado, drift/caminho <=0,015 m durante a pausa ativa e retorno a `Pursuing`/`Navigating` sem novo comando F3. |
| `low-box` | Caixa baixa em `(0,0.35,8)`, half-extents `(1.25,0.35,1.25)`; alvo em Z=18, 900 frames. Exige same-surface detour real, excursão lateral >=1,85 m, passar a caixa, permanecer no chão e chegar/parar. |
| `blocked` | Alvo em `(0,0,18)` fica dentro de caixa sólida centrada em `(0,3,18)`, half-extents `(4,3,3)`; superfície anunciada continua sendo o chão. 900 frames. Aproximar até <=8 m, não explodir/penetrar/perder suporte. Não exige chegar a um alvo impossível nem proíbe transição física válida. |
| `hysteresis` | Alvo inicial em Z=6. Após primeiro `Arrived`, observa 120 frames antes de movê-lo 0,3 m para fora, com jitter lateral de ±0,1 m durante 180 frames. Distância deve ficar estritamente na banda (1,0;1,5) e o corpo permanecer `Arrived`, sem movimento nem intenção. Depois move o alvo a 4 m do corpo, exige `Pursuing`/`Navigating` no primeiro update e uma nova chegada. 1080 frames. |
| `target-lost-reacquire` | Após pelo menos 60 frames, espera movimento físico >0,05 m/s durante pursuit; chama `ClearPursuitTarget()` uma única vez e não publica alvo por 180 frames. Exige ausência de alvo **e** superfície, `WaitingForSurface`, plano vazio, F3/Enabled ainda ON e parada. Reaquisição em `(6,0,18)` sem toggles deve retomar imediatamente e chegar/parar. 1080 frames. |
| `target-lost-reacquire-boundary` | Mesmo protocolo, mas perda fixa no frame 60: pursuit ainda pede ~8 m/s enquanto o gait tem uma pausa física transitória. Reaquisição no frame 240, preservando a trajetória determinística que revelou um problema de precisão no estado de chegada. Não exige velocidade física positiva nesse instante: exige progresso anterior >=1 m, intenção positiva e pursuit ativo. Todas as demais assertions são idênticas. |
| `detour-retarget` | Mesma caixa baixa. Espera pelo menos 15 frames de detour real e deslocamento lateral >=0,25 m; troca o alvo para X=±8 no lado oposto, Z=18. Exige `Navigating` antes da troca, invalidação do plano/cache antigos no primeiro update, nenhum plano apontando para o alvo anterior, lado de detour consistente por alvo e nova chegada. Uma mudança de referência lateral é permitida exatamente no retarget. 1080 frames. |
| `arrived-clear-near` | Chega ao alvo inicial em Z=6, sustenta `Arrived` por 120 frames e chama `ClearPursuitTarget()` uma vez. Observa 60 updates sem alvo, exigindo `WaitingForSurface`, ausência de alvo/superfície/plano e parada. Publica novo alvo a 1,3 m do corpo, sem toggles de F3/Enabled; exige `Pursuing`/`Navigating` imediatamente, aproximação real e nova chegada, não reutilização do latch antigo. 600 frames. |
| `elevated-near-state` | Depois do settling no chão, insere uma plataforma física estreita com topo Y=1,2 e centro Z=0,9. Um raycast real de cima resolve ponto, normal +Y e BodyID desse suporte, publicados pelas APIs de pursuit. Executa um update medido: deve continuar comprovadamente no chão Y=0, com corpo em `BodyClearance`, e não pode reportar `Arrived` por estar a apenas 0,9 m em XZ. Não exige transição completa nem chegada ao piso elevado. |

Para todos: oito pernas válidas no settling, >=4 apoios, nenhuma pose/velocidade não-finita, nenhum frame sem superfície, norma da velocidade <=9,55 m/s, deslocamento por frame <=`9.5*dt + 0.015`, nenhuma penetração de cápsula na caixa >0,015 m. A checagem geométrica independente calcula a distância do segmento central da cápsula ao AABB, considerando orientação física, fora da medição de CPU.

Chegada dos cenários aplicáveis: distância **tangencial XZ** <=1,25 m durante **todos os 120 frames finais**; velocidade máxima <=0,05 m/s, drift e caminho acumulado <=0,015 m nessa janela. Caminho acumulado detecta vai-e-vem mesmo quando o deslocamento líquido é pequeno. O gate agora também exige `Arrived` em **todos** esses 120 frames, não apenas corpo parado perto do alvo. Nos novos lifecycle cases, a chegada final também deve estar dentro de `PursuitStopDistance + ContactToleranceWorld` (1,015 m), e a retomada deve produzir `Pursuing` ou `Navigating` no primeiro update.

Na regressão `arrived-clear-near`, o novo alvo começa a 1,3 m, fora do stop e dentro da antiga banda de resume. A assertion específica exige aproximação até **1,01 + ContactToleranceWorld = 1,025 m**, caminho após republicação >=**0,275 m** e uma **nova** transição para `Arrived`, posterior ao primeiro update de retomada. A assertion compartilhada de lifecycle continua ainda mais estrita, <=**1,015 m**, e os 120 frames finais precisam permanecer `Arrived` e parados. O mínimo anterior de 1 m de caminho dos outros lifecycle cases não foi alterado: o caso novo tem seu próprio mínimo compatível com o deslocamento curto solicitado.

`elevated-near-state` mantém as checagens de segurança, clearance, suporte, penetração e budgets do loop comum. O raycast de preparação é feito fora da janela medida, antes de `Patrol.Update` iniciar/resetar o orçamento do solver. A identificação do chão inferior e do suporte elevado é validada explicitamente para a assertion não passar de forma vacuosa. Como o caso tem um único update, seu p95 é somente essa observação, não uma distribuição de benchmark prolongado.

O setup exige o contrato público padrão **stop=1,0 m / resume=1,5 m**. O caso hysteresis exige explicitamente a sequência `Arrived -> Pursuing/Navigating -> Arrived`. A perda exige `WaitingForSurface -> Pursuing/Navigating -> Arrived`. F3 OFF deve voltar a `Resting`/`Patrolling`, sem copiar o player. Não há fallback aceitando `Pursuing` como se fosse `Arrived`.

Budgets: profile com limites positivos e <=**512 shape queries / 192 raycasts por update**; máximo observado <=profile e p95 <=caps. Não basta média/p95 esconder um frame acima do teto. CPU também exige p95 <=16,667 ms e máximo <=250 ms por padrão.

## Validação atual: regressões da revisão (14/14)

O parent corrigiu o reset de `_pursuitArrived` quando não há alvo e reduziu a tolerância de separação entre superfícies/normal para `max(0.1, Clearance*0.2)`. Foram acrescentados somente os dois cenários acima, sem alterar produção nem enfraquecer assertions anteriores.

Build Release: **sucesso**. Execução `--case all`: **exit 0, 14/14 PASS**, sem HARNESS ERROR, com build e run limitados separadamente a **120 s**. `--f3-override` foi omitido e `environment.json` confirma **resume**. `Program.cs` sem erros/warnings nos diagnósticos. Nenhuma fonte monitorada mudou durante a execução (`sources-changed-during-run.json = []`).

Relatórios atuais: `results-review-edges/` (CSV, summary e environment), ignorados pelo Git. Todos os diretórios anteriores foram preservados. Fuse MVID `23cdf331-be67-4a79-a817-16c51de0a70e`, SHA256 `57FAA5FA04825EE1B09B98979FDFE0F725564BF69B4B7397862D275B85F6BBDD`.

| Caso | Resultado | Update p95 | Shape p95 / max | Ray p95 / max |
|---|---|---:|---:|---:|
| stationary | PASS | 2,512 ms | 146 / 344 | 22 / 110 |
| moving | PASS | 1,286 ms | 158 / 326 | 22 / 110 |
| airborne | PASS | 0,512 ms | 146 / 344 | 22 / 110 |
| near-arrival | PASS | 0,196 ms | 50 / 50 | 10 / 10 |
| f3-off | PASS | 0,938 ms | 218 / 458 | 110 / 114 |
| f6-off-f3-on | PASS | 0,718 ms | 200 / 338 | 62 / 110 |
| low-box | PASS | 0,623 ms | 200 / 386 | 66 / 114 |
| blocked | PASS | 1,618 ms | 436 / 512 | 192 / 192 |
| hysteresis | PASS | 0,348 ms | 74 / 338 | 22 / 110 |
| target-lost-reacquire | PASS | 0,466 ms | 140 / 344 | 22 / 110 |
| target-lost-reacquire-boundary | PASS | 0,402 ms | 116 / 326 | 22 / 110 |
| detour-retarget | PASS | 0,759 ms | 200 / 398 | 74 / 110 |
| arrived-clear-near | **PASS** | 0,338 ms | 104 / 326 | 22 / 110 |
| elevated-near-state | **PASS** | 0,667 ms | 50 / 50 | 78 / 78 |

Maior p95 **2,512 ms**, maior update isolado **13,750 ms**. Nenhum caso excedeu os limites inalterados de 16,667 ms de p95 / 250 ms máximo, **512 shapes / 192 raycasts por update**, segurança, apoios ou parada.

### Evidência dos dois novos edges

- `arrived-clear-near`: `105 Arrived -> 225 WaitingForSurface -> 285 Pursuing -> 304 Arrived`. Novo alvo publicado a **1,3000002 m**, após 60 updates sem alvo com caminho/drift/intenção zero. O corpo percorreu **0,2960081 m** depois da republicação e terminou a **1,003992 m**, com os 120 frames finais parados em `Arrived`. Assim, ficar imediatamente parado a 1,3 m usando o latch antigo falharia tanto no estado de retomada quanto na aproximação/caminho.
- `elevated-near-state`: suporte raycast-resolvido em **Y=1,1999999**, separação inicial XZ **0,9 m**, com BodyID/normal da plataforma superior preservados. Após um update, estado **Navigating**, oito apoios, corpo **Y=0,8299999**, suporte físico ainda no piso Y=0. **Zero** frames de `Arrived` no piso inferior. O caso captura o falso positivo sem cobrar travessia de superfície completa.

## Validação anterior Release (12/12, antes dos edges da revisão)

Após a correção do parent no latch float de chegada (tolerância de 1 cm nos raios stop/resume), atualização de `Navigating` durante transição do motor e ancoragem do patrol, o **mesmo harness** foi recompilado em Release e executado nos 12 casos: **exit 0, 12/12 PASS**, sem HARNESS ERROR. Não houve edição de produção, de outros testes nem de `Program.cs` nesta rodada. Build e cada execução foram limitados separadamente a **120 s**.

A opção `--f3-override` foi **omitida nas três execuções**; todos os `environment.json` registram **`F3Override: resume`**. O SHA256 de `Program.cs`, idêntico ao gate anterior de 11/12, é `8D3FB6F9E7FE00A8A3242A7A4F5C3E53E0F0917C14CEBFB89846B2C377CCDED4`: nenhuma assertion foi relaxada. Todos os relatórios `sources-changed-during-run.json` estão vazios (`[]`).

Assembly Fuse: MVID `90165f7f-d39a-42f1-8f86-11515620d63f`, SHA256 `4622A96A96B7E5C97008F62BD9FB5E4A558216EB09C1A545578D0B432250178F`.

| Caso | Resultado | Update p95 | Shape p95 / max | Ray p95 / max |
|---|---|---:|---:|---:|
| stationary | PASS | 2,561 ms | 146 / 344 | 22 / 110 |
| moving | PASS | 0,858 ms | 158 / 326 | 22 / 110 |
| airborne | PASS | 0,701 ms | 146 / 344 | 22 / 110 |
| near-arrival | PASS | 0,206 ms | 50 / 50 | 10 / 10 |
| f3-off | PASS | 1,046 ms | 218 / 434 | 110 / 110 |
| f6-off-f3-on | PASS | 0,790 ms | 200 / 338 | 62 / 110 |
| low-box | PASS | 0,755 ms | 200 / 386 | 66 / 114 |
| blocked | PASS | 2,047 ms | 436 / 512 | 192 / 192 |
| hysteresis | PASS | 0,366 ms | 74 / 338 | 22 / 110 |
| target-lost-reacquire | PASS | 0,467 ms | 140 / 344 | 22 / 110 |
| target-lost-reacquire-boundary | **PASS** | 0,461 ms | 116 / 326 | 22 / 110 |
| detour-retarget | PASS | 0,794 ms | 200 / 398 | 74 / 110 |

Maior p95 entre os 12 casos: **2,561 ms**, abaixo de 16,667 ms. Maior update isolado: **15,455 ms**, abaixo do teto de 250 ms. Máximos globais de queries: **512 shape / 192 raycasts por update**, sem exceder os budgets. Todas as assertions de segurança, suporte, clearance, penetração, parada e estado passaram.

Evidências funcionais finais:

- A regressão de borda foi corrigida: `0 Pursuing -> 60 WaitingForSurface -> 240 Pursuing -> 408 Arrived`. Distância final **1,0061641 m**, dentro da assertion original de 1,015 m, sem comando residual/oscilações nos 120 frames finais. A assertion continua exigindo `Arrived`, sem aceitar `Pursuing` como substituto.
- Perda em movimento: `66 WaitingForSurface -> 246 Pursuing -> 423 Arrived`; 180 frames sem alvo com caminho, drift, velocidade e intenção zerados.
- F6/F3: `0 Disabled -> 120 Pursuing -> 240 Disabled -> 300 Pursuing -> 421 Arrived`. F3 reativa AI; a pausa F6 independente continua funcionando.
- Retarget: `30 Navigating -> 45 Pursuing -> 55 Navigating -> 306 Pursuing -> 441 Arrived`; zero planos stale depois da troca e zero mudanças inconsistentes de lado dentro de um alvo.

### Estabilidade de hysteresis: duas repetições adicionais

Executadas **sequencialmente**, em processos separados, para não disputar CPU entre benchmarks:

```sh
dotnet run --project tests/SpiderAi.Integration/SpiderAi.Integration.csproj -c Release --no-build -- --case hysteresis --output tests/SpiderAi.Integration/results-final-hysteresis-1
dotnet run --project tests/SpiderAi.Integration/SpiderAi.Integration.csproj -c Release --no-build -- --case hysteresis --output tests/SpiderAi.Integration/results-final-hysteresis-2
```

Ambas deram **exit 0 / PASS**. Incluindo o run completo, hysteresis passou **3/3**, com métricas funcionais idênticas:

- `105 Arrived -> 405 Pursuing -> 479 Arrived`; alvo movido na faixa no frame 225.
- Distância ao alvo durante os 180 frames de hold: **1,305135..1,308960 m**, estritamente dentro da banda exigida (1,0;1,5).
- Caminho, drift, velocidade física e intenção durante o hold: **zero**.
- Após alvo saltar a 4 m: retomada no primeiro update, caminho de **2,995895 m**, nova chegada a **1,0041051 m**, parada estável.
- Shape queries p95/max **74/338**, raycasts p95/max **22/110** nas três execuções.

| Execução hysteresis | Update p95 | Update máximo |
|---|---:|---:|
| Dentro de `all` | 0,366 ms | 10,459 ms |
| Isolada 1 | 2,215 ms | 9,987 ms |
| Isolada 2 | 2,157 ms | 8,886 ms |

Os diretórios `results-final`, `results-final-hysteresis-1` e `results-final-hysteresis-2` contêm CSV, summary e metadados completos e continuam ignorados pelo Git. `results-first-fix`, `results-expanded` e demais relatórios anteriores não foram sobrescritos.

## Primeiro baseline executado (histórico, antes do fix)

O baseline desta seção foi executado com o harness anterior, cujo padrão F3/F6 era `observe`. O padrão **atual** é `resume`, e as assertions de estado/lifecycle abaixo foram acrescentadas depois do primeiro fix do parent.

Build Release: **sucesso**. Execução `--case all`, limites padrão: **exit 1**, seis cenários com falhas e dois sem falhas (um deles é somente observação F3/F6). Nenhum HARNESS ERROR; nenhuma fonte monitorada mudou durante a execução. Assembly Fuse carregado: MVID `21cc685b-0662-4ac1-847d-5333da764626`, SHA256 `A55D39F1A412C22A7EAE7EB32149E01009417BE5921155B67B143302C6EF4F76`.

| Caso | Resultado funcional | Distância final | Caminho nos 2 s finais | Velocidade máxima nos 2 s finais | Update p95 | Shape/ray máximos |
|---|---|---:|---:|---:|---:|---:|
| stationary | FAIL: não para / oscila | 0,033 m | 1,037 m | 2,019 m/s | 3,797 ms | 506 / 172 |
| moving | FAIL: não para após alvo parar | 0,124 m | 1,350 m | 5,637 m/s | 2,070 ms | 512 / 172 |
| airborne | FAIL: aproxima sem voar, mas não para | 0,371 m | 0,283 m | 4,344 m/s | 0,970 ms | 512 / 192 |
| near-arrival | FAIL: oscila mesmo começando perto | 1,018 m | 1,974 m | 5,635 m/s | 1,098 ms | 500 / 172 |
| f3-off | PASS: retomou patrol, não copiou alvo | 37,818 m* | — | — | 1,000 ms | 404 / 172 |
| f6-off-f3-on | OBSERVADO: parado | 18,000 m | — | — | 0,166 ms | 50 / 10 |
| low-box | FAIL: sem detour, travado antes da caixa | 11,599 m | 0 m** | 0 m/s** | 0,772 ms | 512 / 192 |
| blocked | FAIL: pico de movimento/velocidade | 4,353 m | — | — | 1,366 ms | 512 / 177 |

\* Distância ao player depois de F3 OFF não é critério de chegada. Patrol usa `Random` real, portanto a trajetória/distância desse caso varia entre execuções.

\** Parado longe do alvo **não** conta como chegada bem-sucedida.

Falhas relevantes, sem edição de produção:

- Estático: drift final 0,055 m e caminho 1,037 m; móvel: drift 1,228 m e caminho 1,350 m; aéreo: drift 0,214 m e caminho 0,283 m; near-arrival: drift 0,737 m e caminho 1,974 m. Cada um falhou nas três assertions STOP (velocidade/drift/caminho), não na proximidade final.
- Low box: nenhum frame de detour, excursão lateral praticamente zero, termina em `(0,1.2694,6.4010)` e fica rejeitando `leg 4: reach/static clearance`. Norma máxima da velocidade **16,362 m/s**, deslocamento máximo de um frame **0,272697 m**, erro máximo de clearance **0,439 m**. Falhou também na chegada/permanência, excursão lateral e passagem da caixa (oito assertions).
- Bloqueado: termina em `(0,0.7519,13.6466)`, distância mínima 4,353 m; norma máxima da velocidade **21,849 m/s**, deslocamento máximo de um frame **0,364156 m**. Duas assertions de segurança falharam. Não houve não-finitos, penetração acima da tolerância nem perda de suporte.
- F6 OFF → F3 ON: `Enabled=False`, `PursuitEnabled=True`, caminho/deslocamento/progresso depois de F3 **zero**. `--f3-override resume` deve tratar isso como falha; o baseline padrão não impõe a escolha do parent.
- Todos os oito cenários respeitaram os budgets duros, p95 e teto máximo de CPU configurados. Assertions não foram aumentadas para ocultar os problemas funcionais.

## Gate ampliado após primeiro fix (histórico: 11/12)

`results-first-fix/summary.json`, produzido pelo parent, foi conferido: **8/8 PASS**, com override `resume`. Esse diretório foi preservado.

Execução anterior à correção do latch float, com 12 casos e `--f3-override` omitido para validar o novo padrão `resume`:

```sh
dotnet build tests/SpiderAi.Integration/SpiderAi.Integration.csproj -c Release --nologo -v minimal
dotnet run --project tests/SpiderAi.Integration/SpiderAi.Integration.csproj -c Release --no-build -- --case all --output tests/SpiderAi.Integration/results-expanded
```

Build Release: **sucesso**. Run: **exit 1, 11/12 PASS**, sem HARNESS ERROR. Build/run limitados separadamente a 120 s. Diagnóstico de `Program.cs`: sem erros ou warnings. Fontes monitoradas não mudaram durante o run. Fuse MVID `2764712e-756f-4cda-83b6-889414288a19`; SHA256 `8FE7E198B99549CA9733510F0852A6CF27920C925A2843E2FF7061B66400191C`.

| Caso | Resultado | Update p95 | Shape p95 / max | Ray p95 / max |
|---|---|---:|---:|---:|
| stationary | PASS | 2,574 ms | 146 / 344 | 22 / 110 |
| moving | PASS | 1,312 ms | 158 / 326 | 22 / 110 |
| airborne | PASS | 0,415 ms | 146 / 344 | 22 / 110 |
| near-arrival | PASS | 0,186 ms | 50 / 50 | 10 / 10 |
| f3-off | PASS | 0,978 ms | 254 / 392 | 110 / 110 |
| f6-off-f3-on | PASS | 0,656 ms | 200 / 338 | 62 / 110 |
| low-box | PASS | 0,703 ms | 200 / 386 | 66 / 114 |
| blocked | PASS | 1,657 ms | 436 / 512 | 192 / 192 |
| hysteresis | PASS | 0,354 ms | 74 / 344 | 22 / 110 |
| target-lost-reacquire | PASS | 0,422 ms | 140 / 344 | 22 / 110 |
| target-lost-reacquire-boundary | **FAIL: estado não chega a Arrived** | 0,415 ms | 116 / 326 | 22 / 110 |
| detour-retarget | PASS | 0,721 ms | 200 / 398 | 74 / 110 |

Nenhum limite de segurança/performance foi relaxado. Todos os 12 casos passaram nas assertions originais de segurança, budgets e CPU; maior p95 de update 2,574 ms, maior update isolado 13,171 ms. Nos casos de chegada, o corpo ficou parado nos 120 frames finais; o novo estado público expôs a falha que a geometria sozinha não detectava.

### Evidência de estados e lifecycle

- Hysteresis: `106 Arrived -> 406 Pursuing -> 481 Arrived`. Alvo perturbado no frame 226, distância 1,3000002..1,3038406 m por 180 frames; caminho/drift/velocidade/intenção **zero** nessa janela. Depois do salto a 4 m, percorre 3 m e chega novamente.
- Perda em movimento: `66 WaitingForSurface -> 246 Pursuing -> 424 Arrived`. No clear havia velocidade física ~6,60 m/s e desejada ~8,80 m/s. Os 180 frames sem alvo tiveram caminho/drift/intenção **zero**. Reaquisição não altera F3/Enabled e percorre mais 15,286 m.
- F6/F3: `0 Disabled -> 120 Pursuing -> 240 Disabled -> 300 Pursuing -> 423 Arrived`. Pausa independente mantém F3 habilitado, sem movimento/intenção.
- Retarget: `30 Navigating -> 45 Pursuing -> 55 Navigating -> 306 Pursuing -> 443 Arrived`. Troca em pleno detour no frame 45, após 15 frames de plano e X=-0,272 m. Novo alvo `(8,0,18)`; **zero** frames de plano antigo depois da troca, **zero** mudanças inconsistentes de lado dentro de um mesmo alvo. Fim a 1 m do novo alvo.

### Falha histórica de borda de precisão (corrigida na validação final)

A primeira expansão tinha apenas a perda fixa no frame 60 e expôs este problema. Em vez de descartar a reprodução ao tornar a perda principal event-driven, ela foi mantida como **`target-lost-reacquire-boundary` no próprio `all`**.

O caso de regressão continua disponível isoladamente; no assembly final agora retorna PASS. O diretório abaixo preserva a reprodução histórica e não foi sobrescrito:

```sh
dotnet run --project tests/SpiderAi.Integration/SpiderAi.Integration.csproj -c Release --no-build -- --case target-lost-reacquire-boundary --output tests/SpiderAi.Integration/results-boundary-new-run
```

Traço no assembly anterior: `0 Pursuing -> 60 WaitingForSurface -> 240 Pursuing`, **nunca Arrived** até o frame 1079. Estado final anterior:

- Position `(5.6338763, 0.8299999, 17.069433)`; target `(6,0,18)`.
- Distância tangencial **1.0000005 m**; velocidade física **0**.
- Command speed **2.8610228e-5 m/s**, desired speed **2.8610233e-5 m/s**.
- Oito apoios, sem bloqueio, sem plano e sem erro de clearance/segurança.

Duas assertions falhavam: `STATE: final 120 frames must consistently report Arrived` e `LIFECYCLE: replacement target must cause movement and a new Arrived`. A condição estrita de chegada em 1,0 m e a posição sem avanço com comando microscópico apontavam para a borda de arredondamento/latch no Patrol. O parent corrigiu produção; o harness manteve as duas assertions intactas e agora ambas passam.

Relatórios intermediários também foram preservados em diretórios ignorados: `results-expanded-initial` (primeira exposição) e `results-expanded-11` (variante de perda em movimento, antes de adicionar a reprodução de borda ao gate).

## Limites

Este teste chama as APIs públicas correspondentes a F3/F6, **não** sintetiza input GLFW nem executa `DevShortcuts.HandleInput`. A publicação do alvo por `EnemySystem`/`SpiderTargetSurfaceResolver`, hitboxes, dano, áudio e render skin não fazem parte deste harness. O parent deve validar esses caminhos separadamente. A política para alvo aéreo aqui é aproximar no plano físico atual, sem inventar transição para uma superfície que não foi fornecida. O `all` atual não força uma transição entre superfícies nem é um teste prolongado de confinamento ao `PatrolRadius`; portanto, o PASS não valida sozinho essas duas mudanças do parent. Validação visual no jogo continua necessária.
