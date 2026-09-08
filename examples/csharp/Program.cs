using System;
using Grpc.Net.Client;
using Sawyer.Control.V1;

var address = args.Length > 0 ? args[0] : "http://127.0.0.1:50051";
using var channel = GrpcChannel.ForAddress(address);
var client = new RobotControl.RobotControlClient(channel);
var state = await client.GetStateAsync(new Empty());
Console.WriteLine($"enabled={state.Enabled} stopped={state.Stopped}");
Console.WriteLine(string.Join(" ", state.Positions.Values));
