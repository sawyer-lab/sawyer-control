#include <grpcpp/grpcpp.h>
#include <iostream>
#include <memory>

#include "sawyer_control/v1/control.grpc.pb.h"

int main(int argc, char** argv) {
  const std::string address = argc > 1 ? argv[1] : "127.0.0.1:50051";
  auto channel = grpc::CreateChannel(address, grpc::InsecureChannelCredentials());
  auto client = sawyer::control::v1::RobotControl::NewStub(channel);
  grpc::ClientContext context;
  sawyer::control::v1::Empty request;
  sawyer::control::v1::RobotState state;
  const auto status = client->GetState(&context, request, &state);
  if (!status.ok()) {
    std::cerr << status.error_message() << '\n';
    return 1;
  }
  std::cout << "enabled=" << state.enabled() << " stopped=" << state.stopped() << '\n';
  for (double joint : state.positions().values()) std::cout << joint << ' ';
  std::cout << '\n';
}
