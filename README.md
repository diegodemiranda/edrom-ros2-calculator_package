# Exemplo de Cliente e Servidor em ROS2 com Python 🤖

Este repositório demonstra a comunicação síncrona (requisição/resposta) em ROS2 através de um sistema cliente-servidor simples. Diferente dos tópicos (comunicação assíncrona), os serviços são ideais para tarefas que exigem uma confirmação ou um resultado direto.

O sistema é composto por dois nós:

1.  **`calculator_server` (O Servidor):** Um nó que oferece um serviço chamado `/add_two_ints`. Ele aguarda por uma requisição contendo dois números inteiros, os soma e retorna o resultado.
2.  **`calculator_client` (O Cliente):** Um nó que envia uma requisição para o serviço `/add_two_ints` com dois números inteiros. Ele espera pela resposta do servidor e imprime o resultado no console.

Este projeto utiliza o tipo de serviço `AddTwoInts`, que já vem definido no pacote `example_interfaces` do ROS2.

## Pré-requisitos

* Ubuntu 22.04 (ou compatível)
* ROS2 Humble Hawksbill (ou uma versão mais recente)
* Ferramentas de construção `colcon`

## Estrutura do Projeto

O projeto está organizado como um pacote ROS2 do tipo `ament_python`, seguindo a estrutura padrão.

```
ros2_ws/
└── src/
    └── py_srvcli/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── py_srvcli/
            ├── __init__.py
            ├── calculator_server.py  # Nó servidor
            └── calculator_client.py  # Nó cliente
```

## Guia de Instalação e Execução

### 1. Crie um Workspace ROS2

Se você ainda não tiver um, crie um workspace e a pasta `src`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2. Crie o Pacote do Projeto

Dentro da pasta `src`, crie um novo pacote Python para este projeto. É crucial adicionar a dependência `example_interfaces`, que contém a definição do serviço `AddTwoInts`.

```bash
cd src
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

Isso criará uma pasta `py_srvcli` com a estrutura necessária.

### 3. Adicione os Arquivos de Código

Coloque os arquivos `calculator_server.py` e `calculator_client.py` (cujos códigos estão na seção "Código Fonte" abaixo) dentro da pasta `py_srvcli/py_srvcli/`.

Edite o arquivo `setup.py` na raiz do pacote (`py_srvcli/`) para registrar os executáveis dos nós.

### 4. Construa o Pacote

Volte para a raiz do seu workspace (`~/ros2_ws`) e use `colcon` para construir o novo pacote:

```bash
cd ~/ros2_ws
colcon build --packages-select py_srvcli
```

### 5. Configure o Ambiente

Após a construção, "fonte" o ambiente para que o ROS2 reconheça os novos executáveis:

```bash
source install/setup.bash
```

### 6. Execute os Nós

Você precisará de dois terminais. A ordem de execução é importante: o servidor deve estar rodando antes que o cliente possa fazer uma requisição.

* **Terminal 1: Execute o `calculator_server`**

    Abra um novo terminal, fonte o ambiente e inicie o servidor. Ele ficará aguardando por requisições.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run py_srvcli server
    ```

    A saída esperada será:
    ```
    [INFO] [calculator_server]: Servidor de soma pronto.
    ```

* **Terminal 2: Execute o `calculator_client`**

    Abra um segundo terminal, fonte o ambiente e execute o cliente. Você pode passar os dois números a serem somados como argumentos de linha de comando.
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run py_srvcli client 5 12
    ```

    O cliente enviará a requisição, receberá a resposta e a imprimirá no console antes de finalizar. A saída esperada será:
    ```
    [INFO] [calculator_client]: Requisição enviada: 5 + 12
    [INFO] [calculator_client]: Resultado: 5 + 12 = 17
    ```

    No **Terminal 1 (servidor)**, você verá uma mensagem confirmando que a requisição foi recebida e processada:
    ```
    [INFO] [calculator_server]: Requisição recebida: 5 + 12. Retornando 17.
    ```

Você pode executar o cliente novamente com números diferentes sem precisar reiniciar o servidor.
