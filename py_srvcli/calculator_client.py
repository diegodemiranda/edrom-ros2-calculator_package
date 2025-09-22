import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class CalculatorClientNode(Node):
    """
    Um nó cliente que envia uma requisição para somar dois inteiros.
    """
    def __init__(self):
        super().__init__('calculator_client')
        # Cria um cliente para o serviço /add_two_ints
        # O tipo e o nome devem ser os mesmos do servidor
        self.client = self.create_client(AddTwoInts, '/add_two_ints')
        
        # Aguarda em um loop até que o servidor esteja disponível
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço não disponível, esperando novamente...')
        
        # Cria um objeto de requisição vazio
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Define os valores da requisição e a envia de forma assíncrona.
        """
        self.req.a = a
        self.req.b = b
        # call_async envia a requisição e retorna um objeto 'future' imediatamente
        self.future = self.client.call_async(self.req)
        self.get_logger().info(f'Requisição enviada: {a} + {b}')

def main(args=None):
    rclpy.init(args=args)
    client_node = CalculatorClientNode()

    # Verifica se os argumentos da linha de comando foram passados corretamente
    if len(sys.argv) != 3:
        client_node.get_logger().error('Uso: ros2 run py_srvcli client <int1> <int2>')
        client_node.destroy_node()
        rclpy.shutdown()
        return

    # Envia a requisição com os números passados como argumento
    try:
        num1 = int(sys.argv[1])
        num2 = int(sys.argv[2])
    except ValueError:
        client_node.get_logger().error('Ambos os argumentos devem ser números inteiros.')
        client_node.destroy_node()
        rclpy.shutdown()
        return
        
    client_node.send_request(num1, num2)

    # Fica em um loop (spin_once) até que a resposta (future) seja completada
    while rclpy.ok():
        rclpy.spin_once(client_node)
        if client_node.future.done():
            try:
                response = client_node.future.result()
            except Exception as e:
                client_node.get_logger().error(f'Falha ao chamar serviço: {e}')
            else:
                client_node.get_logger().info(
                    f'Resultado: {client_node.req.a} + {client_node.req.b} = {response.sum}')
            break  # Sai do loop após receber a resposta

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
