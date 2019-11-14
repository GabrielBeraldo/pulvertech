***
<h1>firmware todo</h1> 

## Escrever algoritmos para calibração baseado no sinal da IHM
- Rotina de inicialização 
- captação da informação
- atualização do valor na EEPROM
- modo standalone (em caso de problema na comunicação com a tela)

## comunicação com ihm(depende interface gráfica)
- Rotina de inicialização
- Rotina de atualização
- Captura de dados
- envio dos dados
- geração de logs no sd


## adequar leitura de variáveis para os cálculos
- revisar interruptores
- revisar os filtros de entrada
- revisar os filtros de saida
- revisar parametros filtro de kalman
- limitar valor das variáveis
	
## alterar algorítmo controlador(controlar retorno)
- inverter acionamento da válvula para controlar o fluxo do retorno
- ajustar ganhos PID

## refinamentos gerais
- variaveis para telemetria
- eliminar trechos inutilizados
- conferir a necessidade dos tipos de variável
- conferir parametros das funções(adequar aos tipos das variáveis)

***