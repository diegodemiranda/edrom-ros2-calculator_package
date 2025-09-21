import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class CalculatorServerNode(Node):
    """
    Um nó servidor que oferece um serviço para somar dois inteiros.
    """
    def __init__(self):
        super().__init__('calculator_server')
        # Cria o serviço com tipo AddTwoInts, nome /add_two_ints e define a função de callback
        self.srv = self.create_service(
            AddTwoInts, '/add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Servidor de soma pronto.')

    def add_two_ints_callback(self, request, response):
        """
        Callback que processa a requisição do serviço.
        O objeto 'request' contém os dados de entrada (a, b).
        O objeto 'response' deve ser preenchido com o resultado (sum).
        """
        # Soma os dois inteiros da requisição
        response.sum = request.a + request.b
        
        # Loga a operação no console do servidor
        self.get_logger().info(
            f'Requisição recebida: {request.a} + {request.b}. Retornando {response.sum}.')
        
        # Retorna o objeto de resposta modificado
        return response

def main(args=None):
    rclpy.init(args=args)
    server_node = CalculatorServerNode()
    rclpy.spin(server_node)  # Mantém o servidor rodando e aguardando requisições
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
