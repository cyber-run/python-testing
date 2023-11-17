import MoCapStream

telemetry = MoCapStream.MoCap(stream_type='3d_unlabelled')
while True:
    print(telemetry.position)
