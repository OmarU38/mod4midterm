package object_detection;

import static io.grpc.MethodDescriptor.generateFullMethodName;

/**
 */
@javax.annotation.Generated(
    value = "by gRPC proto compiler (version 1.42.1)",
    comments = "Source: cx100.proto")
@io.grpc.stub.annotations.GrpcGenerated
public final class GreenObjectServiceGrpc {

  private GreenObjectServiceGrpc() {}

  public static final String SERVICE_NAME = "object_detection.GreenObjectService";

  // Static method descriptors that strictly reflect the proto.
  private static volatile io.grpc.MethodDescriptor<object_detection.Cx100.Empty,
      object_detection.Cx100.ObjectCoordinates> getPublishObjectCoordinatesMethod;

  @io.grpc.stub.annotations.RpcMethod(
      fullMethodName = SERVICE_NAME + '/' + "PublishObjectCoordinates",
      requestType = object_detection.Cx100.Empty.class,
      responseType = object_detection.Cx100.ObjectCoordinates.class,
      methodType = io.grpc.MethodDescriptor.MethodType.UNARY)
  public static io.grpc.MethodDescriptor<object_detection.Cx100.Empty,
      object_detection.Cx100.ObjectCoordinates> getPublishObjectCoordinatesMethod() {
    io.grpc.MethodDescriptor<object_detection.Cx100.Empty, object_detection.Cx100.ObjectCoordinates> getPublishObjectCoordinatesMethod;
    if ((getPublishObjectCoordinatesMethod = GreenObjectServiceGrpc.getPublishObjectCoordinatesMethod) == null) {
      synchronized (GreenObjectServiceGrpc.class) {
        if ((getPublishObjectCoordinatesMethod = GreenObjectServiceGrpc.getPublishObjectCoordinatesMethod) == null) {
          GreenObjectServiceGrpc.getPublishObjectCoordinatesMethod = getPublishObjectCoordinatesMethod =
              io.grpc.MethodDescriptor.<object_detection.Cx100.Empty, object_detection.Cx100.ObjectCoordinates>newBuilder()
              .setType(io.grpc.MethodDescriptor.MethodType.UNARY)
              .setFullMethodName(generateFullMethodName(SERVICE_NAME, "PublishObjectCoordinates"))
              .setSampledToLocalTracing(true)
              .setRequestMarshaller(io.grpc.protobuf.ProtoUtils.marshaller(
                  object_detection.Cx100.Empty.getDefaultInstance()))
              .setResponseMarshaller(io.grpc.protobuf.ProtoUtils.marshaller(
                  object_detection.Cx100.ObjectCoordinates.getDefaultInstance()))
              .setSchemaDescriptor(new GreenObjectServiceMethodDescriptorSupplier("PublishObjectCoordinates"))
              .build();
        }
      }
    }
    return getPublishObjectCoordinatesMethod;
  }

  /**
   * Creates a new async stub that supports all call types for the service
   */
  public static GreenObjectServiceStub newStub(io.grpc.Channel channel) {
    io.grpc.stub.AbstractStub.StubFactory<GreenObjectServiceStub> factory =
      new io.grpc.stub.AbstractStub.StubFactory<GreenObjectServiceStub>() {
        @java.lang.Override
        public GreenObjectServiceStub newStub(io.grpc.Channel channel, io.grpc.CallOptions callOptions) {
          return new GreenObjectServiceStub(channel, callOptions);
        }
      };
    return GreenObjectServiceStub.newStub(factory, channel);
  }

  /**
   * Creates a new blocking-style stub that supports unary and streaming output calls on the service
   */
  public static GreenObjectServiceBlockingStub newBlockingStub(
      io.grpc.Channel channel) {
    io.grpc.stub.AbstractStub.StubFactory<GreenObjectServiceBlockingStub> factory =
      new io.grpc.stub.AbstractStub.StubFactory<GreenObjectServiceBlockingStub>() {
        @java.lang.Override
        public GreenObjectServiceBlockingStub newStub(io.grpc.Channel channel, io.grpc.CallOptions callOptions) {
          return new GreenObjectServiceBlockingStub(channel, callOptions);
        }
      };
    return GreenObjectServiceBlockingStub.newStub(factory, channel);
  }

  /**
   * Creates a new ListenableFuture-style stub that supports unary calls on the service
   */
  public static GreenObjectServiceFutureStub newFutureStub(
      io.grpc.Channel channel) {
    io.grpc.stub.AbstractStub.StubFactory<GreenObjectServiceFutureStub> factory =
      new io.grpc.stub.AbstractStub.StubFactory<GreenObjectServiceFutureStub>() {
        @java.lang.Override
        public GreenObjectServiceFutureStub newStub(io.grpc.Channel channel, io.grpc.CallOptions callOptions) {
          return new GreenObjectServiceFutureStub(channel, callOptions);
        }
      };
    return GreenObjectServiceFutureStub.newStub(factory, channel);
  }

  /**
   */
  public static abstract class GreenObjectServiceImplBase implements io.grpc.BindableService {

    /**
     */
    public void publishObjectCoordinates(object_detection.Cx100.Empty request,
        io.grpc.stub.StreamObserver<object_detection.Cx100.ObjectCoordinates> responseObserver) {
      io.grpc.stub.ServerCalls.asyncUnimplementedUnaryCall(getPublishObjectCoordinatesMethod(), responseObserver);
    }

    @java.lang.Override public final io.grpc.ServerServiceDefinition bindService() {
      return io.grpc.ServerServiceDefinition.builder(getServiceDescriptor())
          .addMethod(
            getPublishObjectCoordinatesMethod(),
            io.grpc.stub.ServerCalls.asyncUnaryCall(
              new MethodHandlers<
                object_detection.Cx100.Empty,
                object_detection.Cx100.ObjectCoordinates>(
                  this, METHODID_PUBLISH_OBJECT_COORDINATES)))
          .build();
    }
  }

  /**
   */
  public static final class GreenObjectServiceStub extends io.grpc.stub.AbstractAsyncStub<GreenObjectServiceStub> {
    private GreenObjectServiceStub(
        io.grpc.Channel channel, io.grpc.CallOptions callOptions) {
      super(channel, callOptions);
    }

    @java.lang.Override
    protected GreenObjectServiceStub build(
        io.grpc.Channel channel, io.grpc.CallOptions callOptions) {
      return new GreenObjectServiceStub(channel, callOptions);
    }

    /**
     */
    public void publishObjectCoordinates(object_detection.Cx100.Empty request,
        io.grpc.stub.StreamObserver<object_detection.Cx100.ObjectCoordinates> responseObserver) {
      io.grpc.stub.ClientCalls.asyncUnaryCall(
          getChannel().newCall(getPublishObjectCoordinatesMethod(), getCallOptions()), request, responseObserver);
    }
  }

  /**
   */
  public static final class GreenObjectServiceBlockingStub extends io.grpc.stub.AbstractBlockingStub<GreenObjectServiceBlockingStub> {
    private GreenObjectServiceBlockingStub(
        io.grpc.Channel channel, io.grpc.CallOptions callOptions) {
      super(channel, callOptions);
    }

    @java.lang.Override
    protected GreenObjectServiceBlockingStub build(
        io.grpc.Channel channel, io.grpc.CallOptions callOptions) {
      return new GreenObjectServiceBlockingStub(channel, callOptions);
    }

    /**
     */
    public object_detection.Cx100.ObjectCoordinates publishObjectCoordinates(object_detection.Cx100.Empty request) {
      return io.grpc.stub.ClientCalls.blockingUnaryCall(
          getChannel(), getPublishObjectCoordinatesMethod(), getCallOptions(), request);
    }
  }

  /**
   */
  public static final class GreenObjectServiceFutureStub extends io.grpc.stub.AbstractFutureStub<GreenObjectServiceFutureStub> {
    private GreenObjectServiceFutureStub(
        io.grpc.Channel channel, io.grpc.CallOptions callOptions) {
      super(channel, callOptions);
    }

    @java.lang.Override
    protected GreenObjectServiceFutureStub build(
        io.grpc.Channel channel, io.grpc.CallOptions callOptions) {
      return new GreenObjectServiceFutureStub(channel, callOptions);
    }

    /**
     */
    public com.google.common.util.concurrent.ListenableFuture<object_detection.Cx100.ObjectCoordinates> publishObjectCoordinates(
        object_detection.Cx100.Empty request) {
      return io.grpc.stub.ClientCalls.futureUnaryCall(
          getChannel().newCall(getPublishObjectCoordinatesMethod(), getCallOptions()), request);
    }
  }

  private static final int METHODID_PUBLISH_OBJECT_COORDINATES = 0;

  private static final class MethodHandlers<Req, Resp> implements
      io.grpc.stub.ServerCalls.UnaryMethod<Req, Resp>,
      io.grpc.stub.ServerCalls.ServerStreamingMethod<Req, Resp>,
      io.grpc.stub.ServerCalls.ClientStreamingMethod<Req, Resp>,
      io.grpc.stub.ServerCalls.BidiStreamingMethod<Req, Resp> {
    private final GreenObjectServiceImplBase serviceImpl;
    private final int methodId;

    MethodHandlers(GreenObjectServiceImplBase serviceImpl, int methodId) {
      this.serviceImpl = serviceImpl;
      this.methodId = methodId;
    }

    @java.lang.Override
    @java.lang.SuppressWarnings("unchecked")
    public void invoke(Req request, io.grpc.stub.StreamObserver<Resp> responseObserver) {
      switch (methodId) {
        case METHODID_PUBLISH_OBJECT_COORDINATES:
          serviceImpl.publishObjectCoordinates((object_detection.Cx100.Empty) request,
              (io.grpc.stub.StreamObserver<object_detection.Cx100.ObjectCoordinates>) responseObserver);
          break;
        default:
          throw new AssertionError();
      }
    }

    @java.lang.Override
    @java.lang.SuppressWarnings("unchecked")
    public io.grpc.stub.StreamObserver<Req> invoke(
        io.grpc.stub.StreamObserver<Resp> responseObserver) {
      switch (methodId) {
        default:
          throw new AssertionError();
      }
    }
  }

  private static abstract class GreenObjectServiceBaseDescriptorSupplier
      implements io.grpc.protobuf.ProtoFileDescriptorSupplier, io.grpc.protobuf.ProtoServiceDescriptorSupplier {
    GreenObjectServiceBaseDescriptorSupplier() {}

    @java.lang.Override
    public com.google.protobuf.Descriptors.FileDescriptor getFileDescriptor() {
      return object_detection.Cx100.getDescriptor();
    }

    @java.lang.Override
    public com.google.protobuf.Descriptors.ServiceDescriptor getServiceDescriptor() {
      return getFileDescriptor().findServiceByName("GreenObjectService");
    }
  }

  private static final class GreenObjectServiceFileDescriptorSupplier
      extends GreenObjectServiceBaseDescriptorSupplier {
    GreenObjectServiceFileDescriptorSupplier() {}
  }

  private static final class GreenObjectServiceMethodDescriptorSupplier
      extends GreenObjectServiceBaseDescriptorSupplier
      implements io.grpc.protobuf.ProtoMethodDescriptorSupplier {
    private final String methodName;

    GreenObjectServiceMethodDescriptorSupplier(String methodName) {
      this.methodName = methodName;
    }

    @java.lang.Override
    public com.google.protobuf.Descriptors.MethodDescriptor getMethodDescriptor() {
      return getServiceDescriptor().findMethodByName(methodName);
    }
  }

  private static volatile io.grpc.ServiceDescriptor serviceDescriptor;

  public static io.grpc.ServiceDescriptor getServiceDescriptor() {
    io.grpc.ServiceDescriptor result = serviceDescriptor;
    if (result == null) {
      synchronized (GreenObjectServiceGrpc.class) {
        result = serviceDescriptor;
        if (result == null) {
          serviceDescriptor = result = io.grpc.ServiceDescriptor.newBuilder(SERVICE_NAME)
              .setSchemaDescriptor(new GreenObjectServiceFileDescriptorSupplier())
              .addMethod(getPublishObjectCoordinatesMethod())
              .build();
        }
      }
    }
    return result;
  }
}
