# Regressão de matemática e IK da aranha

Harness console independente, sem pacotes NuGet nem framework de testes. Requer
Windows e SDK .NET 10. Não é necessário inicializar OpenGL/Jolt, importar assets
ou compilar `Fuse`/`Blowtorch`.

Na raiz do repositório:

```sh
dotnet run --project tests/SpiderLocomotion.Tests/SpiderLocomotion.Tests.csproj
dotnet run --project tests/SpiderLocomotion.Tests/SpiderLocomotion.Tests.csproj --configuration Release
```

Cada caso imprime `PASS` ou `FAIL`; falhas incluem contexto e não interrompem os
outros casos. O resumo informa o total. Código de saída: `0` se todos passarem,
`1` se houver falha. Use `dotnet run`, não `dotnet test`.

## Isolamento

O `.csproj` compila as fontes reais por `Compile Include` com `Link`:

- `Fuse/src/Animation/SpiderLocomotionMath.cs`
- `Fuse/src/Animation/SpiderLegKinematics.cs`
- `Fuse/src/Enemy/SpiderLocomotionProfile.cs`

Não há cópias das implementações, `ProjectReference`, `PackageReference` ou
alteração da solution. Mudanças nessas fontes são exercitadas na próxima
execução. Os únicos artefatos de build ficam em `bin/` e `obj/` desta pasta.
Os tipos `internal` ficam acessíveis porque são compilados no mesmo assembly.

## Cobertura

- **ContainsSupport:** piso, duas orientações de parede e teto; projeção normal,
  pontos desordenados/repetidos/interiores, arestas/vértices inclusivos, margens
  positivas/zero/negativas, triângulo versus AABB, conjuntos vazios/insuficientes,
  coincidentes, colineares e degenerados apenas após projeção.
- **StepPoint:** endpoints e clamp de `t`, altura central, simetria e progresso
  no plano, lift zero e contato estacionário; amostragem de posição/velocidade
  e velocidade nula nos extremos em piso e transições para parede/teto;
  fallback de normais nulas. As verificações de suavidade são numéricas, não
  uma prova analítica de continuidade nem uma exigência de aceleração nula.
- **RotationBetween:** opostos com eixo preferido válido, paralelo ou nulo,
  vetores não unitários, identidade, quarto de volta e quase opostos. Verifica
  quaternion unitário/finito e a transformação, sem exigir um sinal específico.
- **AngularVelocity:** todas as combinações de sinais de `q`/`-q`, orientação
  inicial não trivial para conferir o eixo no espaço global, menor arco,
  meia-volta, movimento minúsculo e `dt` nulo/negativo/abaixo do limiar.
- **IK:** rigs sintéticos de dois/três segmentos; comprimentos, hip fixo e
  contato sem clamp; todos os limites geométricos de poses aceitas; alvos
  alcançáveis, além do alcance e singularidades interna/externa; rejeição por
  limites com controle alcançável sob limites amplos; pitch/plano travados e
  poses anteriores incompatíveis com limites mais restritos; callback de
  aceitação total/parcial/rejeição; entradas não finitas e segmentos proximais
  nulos. Inclui translação, yaw e espelhamento da calibração.
- **Continuidade do IK:** 361 amostras por trajetória fechada pequena para cada
  rig e 300 resoluções de contato fixo fora do repouso, verificando saltos,
  deriva acumulada e a validade da solução anterior no próximo frame.

Asserções de limites usam geometria independente (`atan2` de produto vetorial
versus escalar), sem chamar o predicado privado do solver ou `Angle`/`Normal`
da produção. Tolerâncias explícitas em `Program.cs` cobrem arredondamento de
`float`: comprimentos `3e-5`, contato `2e-6`, ângulos `0.003` grau e deriva
acumulada `2e-4`, em unidades sintéticas. Limites de deslocamento entre amostras
são proporcionais à pequena trajetória escolhida, não a uma marcha real.

## Limites do escopo

Os rigs são fixtures matemáticas deliberadamente simples, não uma anatomia de
aranha. Os testes não validam ossos, eixos ou escala de um rig importado,
colisões reais, estabilidade da marcha, agendamento de passos ou transições de
superfície do controlador. A cobertura piso/parede/teto é de suporte e trajetória;
não pressupõe invariância arbitrária do IK, que usa `UnitY` na calibração/limites.
Normais de contato do IK são unitárias. Não é exigida uma pose específica quando
há várias soluções válidas, nem sucesso em todo alvo dentro da esfera de alcance
(limites e amostragem de candidatos podem torná-lo inviável).
