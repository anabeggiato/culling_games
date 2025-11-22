# Documentação Técnica — MazeSolverNode

## 1. Objetivo

A classe `MazeSolverNode` tem como objetivo:

1. Solicitar o mapa do ambiente via serviço ROS (`GetMapSrv`).  
2. Receber informações de sensores do robô (`RobotSensorsMsg`).  
3. Calcular o caminho otimizado até o objetivo (`'t'`) usando o algoritmo A*.  
4. Enviar comandos de movimento ao robô via serviço ROS (`MoveCmdSrv`).  

> Parte 1 do desafio: navegação com mapa conhecido e cálculo de caminho.

---

## 2. Estrutura Geral do Nó

### 2.1 Construtor `MazeSolverNode::MazeSolverNode()`

- Cria **clientes ROS**:

```cpp
map_client_ = this->create_client<GetMapSrv>("get_map");
move_client_ = this->create_client<MoveCmdSrv>("move_cmd");
```

- Espera pelo serviço de movimento **uma vez no início**, evitando bloqueio repetitivo:

```cpp
if (!move_client_->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_FATAL(this->get_logger(), "Serviço MoveCmd não ficou disponível após 60s!");
} else {
    RCLCPP_INFO(this->get_logger(), "Serviço /move_command encontrado!");
}
```

- Cria **subscrição** para sensores do robô:

```cpp
sensor_sub_ = this->create_subscription<RobotSensorsMsg>(
    "/culling_games/robot_sensors", 10,
    std::bind(&MazeSolverNode::sensorCallback, this, std::placeholders::_1));
```

- Solicita o mapa chamando `requestMap()`.

---

### 2.2 `requestMap()`

- Envia requisição ao serviço `GetMapSrv` de forma **assíncrona**.  
- Ao receber o mapa:
  - Redimensiona `map_` dinamicamente (`map_.resize(rows, vector<char>(cols))`).  
  - Identifica posição do robô `'r'` e objetivo `'t'`.  
  - Log detalhado do mapa com cores ANSI.  
  - Atualiza `robot_x_, robot_y_` e `goal_x_, goal_y_`.  
- Se não encontrar robô ou alvo, usa posições fallback.

---

### 2.3 `sensorCallback(RobotSensorsMsg::SharedPtr msg)`

- Recebe dados dos sensores (`up`, `down`, `left`, `right`).  
- Verifica se o mapa foi carregado, ignorando callback caso contrário.  
- Verifica se o **alvo foi alcançado**:

```cpp
if (robot_x_ == goal_x && robot_y_ == goal_y) {
    RCLCPP_INFO(this->get_logger(), "🎉 ALVO ALCANÇADO em (%d,%d)!", goal_x, goal_y);
    return;
}
```

- Chama `computeNextMove` para calcular próximo passo.  
- Determina **direção** (`up`, `down`, `left`, `right`).  
- Envia comando via `sendMoveCommand`.

---

### 2.4 `sendMoveCommand`

- Cria `MoveCmdSrv::Request` com a direção.  
- **Verifica se serviço está pronto** (`service_is_ready`).  
- Envia **requisição assíncrona**, atualizando posição interna **apenas se movimento for bem-sucedido**.

---

### 2.5 `computeNextMove`

Implementa **algoritmo A***:

- Estruturas usadas:
  - `std::priority_queue<AStarNode, vector<AStarNode>, CompareAStar>` → open list.  
  - `std::map<pair<int,int>, pair<int,int>> came_from` → rastreamento do caminho.  
  - `std::map<pair<int,int>, int> g_score` → custo até cada nó.  

- Explora vizinhos (`directions`) verificando limites e obstáculos.
- Reconstrói caminho retornando **o próximo passo** a partir da posição atual.
- Se não encontrar caminho, retorna a posição atual.

---

## 3. Ponteiros e Gerenciamento de Memória

| Membro               | Tipo | Gestão |
|----------------------|------|--------|
| `map_client_`        | `SharedPtr` | Automática (cliente ROS) |
| `move_client_`       | `SharedPtr` | Automática (cliente ROS) |
| `sensor_sub_`        | `SharedPtr` | Automática (subscrição ROS) |
| `map_`               | `vector<vector<char>>` | Alocação dinâmica automática pelo STL |
| Caminho no A*        | `priority_queue` + `map` | Alocação automática |

> Nenhum `new` ou `delete` é usado manualmente.

---

## 4. Fluxo do Algoritmo

```text
Construtor -> criar clientes e subscrições
           -> esperar MoveCmd
           -> requestMap()
requestMap() -> recebe mapa
             -> atualiza map_, robot_x/y, goal_x/y
sensorCallback -> verifica sensores
               -> verifica alvo alcançado
               -> computeNextMove
               -> sendMoveCommand
```

---

### 4.1 Fluxo do A* (simplificado)

```text
start
 └─> adicionar start em open_list
      └─> enquanto open_list não vazia:
           ├─> pegar nó com menor f
           ├─> se nó == goal: reconstruir caminho
           └─> explorar vizinhos (up, down, left, right)
                ├─> ignorar parede ou obstáculo
                ├─> calcular g, h, f
                └─> adicionar vizinho em open_list
```

---

## 5. Visualização do Mapa Recebido (Exemplo)

```text
b b b b b b b b b b b
b r f f f f f f f f b
b b b b b b b b f b b
b f f f f f f f b f b
b b b b b f b f b f b
b f f f b f f f b f b
b f b b b b b f b f b
b f f f b f f f b f b
b b b f b f b b b f b
b f f f b f f f b f t
```

Legenda:  
- `r` = robô  
- `t` = alvo  
- `f` = caminho livre  
- `b` = parede  

---

## 6. Saída Observada e Problema

```
[FATAL] [maze_solver]: Serviço MoveCmd não ficou disponível após 60s!
[WARN] [maze_solver]: Serviço MoveCmd não está pronto. Pulando comando.
```

- Serviço de movimento **não ativo ou não registrado corretamente**.  
- Cálculo do caminho funciona, mas o robô **não se move**.

---

## 7. Conclusão da Parte 1

- **A*** implementado corretamente, calcula próximo passo e direção. ✅  
- **Mapeamento e identificação do robô/objetivo** corretos. ✅  
- **Próximo passo e direção calculados** corretamente. ✅  
- **Problema crítico**: comunicação com `/move_command` falha. ❌

**Pontos de destaque técnico:**

- Uso de **SharedPtr** para clientes e subscrições ROS.  
- Uso de **vetores dinâmicos** para mapa e caminho.  
- Logs detalhados para depuração e visualização do mapa.  
- Lógica robusta para **checagem de alvo**, **obstáculos** e **movimento seguro**.

---

