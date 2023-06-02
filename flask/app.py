from flask import Flask, render_template
import json
import grpc
import cx100_pb2
import cx100_pb2_grpc
import time
from google.protobuf.json_format import MessageToJson

app = Flask(__name__,template_folder='templates')

def get_coords():
    channel = grpc.insecure_channel('localhost:50052')
    #REST 
    #Postman
    request = cx100_pb2.Empty()
    # Crear un cliente (stub)
    stub = cx100_pb2_grpc.GreenObjectServiceStub(channel)
    # Crear un mensaje de solicitud
    # Realizar la llamada al método remoto
    response = stub.PublishObjectCoordinates(request)
    print(response)
    return response

# Flask route para realizar la llamada al servidor gRPC
@app.route('/v1/green-object/coordinates')
def call_go_api():
    coords = json.loads(MessageToJson(get_coords()))

    return render_template('index.html', coords=coords)
    # Resto del código Flask
    # ...
    

if __name__ == '__main__':
    app.run()