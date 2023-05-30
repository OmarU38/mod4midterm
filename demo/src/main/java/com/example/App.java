package com.example;

import com.google.protobuf.Empty;

import io.grpc.ManagedChannel;
import io.grpc.ManagedChannelBuilder;
import object_detection.GreenObjectServiceGrpc;
import object_detection.Cx100.ObjectCoordinates;

/**
 * Clase principal que muestra cómo utilizar el cliente gRPC para obtener las coordenadas del objeto verde.
 */
public class App {
    public static void main(String[] args) {
        // Establecer la conexión con el servidor gRPC
        ManagedChannel channel = ManagedChannelBuilder.forAddress("localhost", 50051)
                .usePlaintext()
                .build();

        // Crear el stub del servicio gRPC
        GreenObjectServiceGrpc.GreenObjectServiceBlockingStub stub = GreenObjectServiceGrpc.newBlockingStub(channel);

        try {
            // Crear una instancia de Empty para enviar al servidor
            object_detection.Cx100.Empty empty = object_detection.Cx100.Empty.getDefaultInstance();

            while (true) {
                // Llamar al método del servicio y recibir la respuesta
                ObjectCoordinates response = stub.publishObjectCoordinates(empty);

                // Imprimir las coordenadas recibidas
                System.out.println("X: " + response.getX());
                System.out.println("Y: " + response.getY());
                System.out.println("Timestamp: " + response.getTimestamp());
                System.out.println("---------------------");

                Thread.sleep(100); // Esperar 1 segundo antes de realizar la siguiente llamada
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        } finally {
            // Cerrar el canal al finalizar
            channel.shutdown();
        }
    }
}