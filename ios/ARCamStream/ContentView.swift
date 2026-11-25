import SwiftUI
import ARKit
import Network

// Streams the iPhone camera pose (ARKit visual-inertial tracking) to Blender over UDP.
// Packet: AR1,seq,timestamp,px,py,pz,qx,qy,qz,qw,trackingState   (0 none, 1 limited, 2 normal)

final class PoseSender: NSObject, ObservableObject, ARSessionDelegate {
    @Published var status = "Stopped"
    @Published var tracking = "-"
    @Published var running = false
    @Published var orientIndex = 0          // 0 portrait, 1 landscape left, 2 landscape right

    private let session = ARSession()
    private var connection: NWConnection?
    private var seq: Int32 = 0

    func start(host: String, port: String) {
        stop()
        guard !host.isEmpty, let p = UInt16(port), let nwPort = NWEndpoint.Port(rawValue: p) else {
            status = "Bad IP or port"
            return
        }
        let c = NWConnection(host: NWEndpoint.Host(host), port: nwPort, using: .udp)
        c.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async { self?.status = "UDP: \(state)" }
        }
        c.start(queue: DispatchQueue(label: "udp.send"))
        connection = c
        beginAR(reset: true)
        running = true
        UIApplication.shared.isIdleTimerDisabled = true   // keep screen on
    }

    func beginAR(reset: Bool) {
        let cfg = ARWorldTrackingConfiguration()
        cfg.worldAlignment = .gravity                      // Y up, gravity aligned
        session.delegate = self
        let options: ARSession.RunOptions = reset ? [.resetTracking, .removeExistingAnchors] : []
        session.run(cfg, options: options)
    }

    func resetTracking() { if running { beginAR(reset: true) } }

    func stop() {
        session.pause()
        connection?.cancel()
        connection = nil
        running = false
        UIApplication.shared.isIdleTimerDisabled = false
        status = "Stopped"
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard let connection = connection else { return }
        let cam = frame.camera

        let orientation: UIInterfaceOrientation =
            orientIndex == 1 ? .landscapeLeft : (orientIndex == 2 ? .landscapeRight : .portrait)
        let m = simd_inverse(cam.viewMatrix(for: orientation))   // camera-to-world, screen-oriented
        let p = m.columns.3
        let q = simd_quatf(m)

        var state: Int32 = 0
        var label = "none"
        switch cam.trackingState {
        case .normal:        state = 2; label = "normal"
        case .limited:       state = 1; label = "limited"
        case .notAvailable:  state = 0; label = "none"
        }

        seq &+= 1
        let line = String(format: "AR1,%d,%.6f,%.5f,%.5f,%.5f,%.7f,%.7f,%.7f,%.7f,%d",
                          seq, frame.timestamp,
                          Double(p.x), Double(p.y), Double(p.z),
                          Double(q.imag.x), Double(q.imag.y), Double(q.imag.z), Double(q.real),
                          state)
        connection.send(content: line.data(using: .utf8), completion: .contentProcessed { _ in })

        if seq % 20 == 0 { tracking = label }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        status = "AR error: \(error.localizedDescription)"
    }
}

struct ContentView: View {
    @StateObject private var sender = PoseSender()
    @AppStorage("host") private var host = "192.168.1.10"
    @AppStorage("port") private var port = "9000"

    var body: some View {
        VStack(spacing: 16) {
            Text("ARKit → Blender").font(.title2.bold())

            TextField("Computer IP (shown in Blender panel)", text: $host)
                .textFieldStyle(.roundedBorder)
                .keyboardType(.numbersAndPunctuation)
                .textInputAutocapitalization(.never)
                .autocorrectionDisabled()
            TextField("Port", text: $port)
                .textFieldStyle(.roundedBorder)
                .keyboardType(.numberPad)

            Picker("Holding", selection: $sender.orientIndex) {
                Text("Portrait").tag(0)
                Text("Landscape L").tag(1)
                Text("Landscape R").tag(2)
            }.pickerStyle(.segmented)

            Button(sender.running ? "Stop" : "Start streaming") {
                sender.running ? sender.stop() : sender.start(host: host, port: port)
            }
            .buttonStyle(.borderedProminent)
            .tint(sender.running ? .red : .green)

            Button("Reset tracking") { sender.resetTracking() }
                .buttonStyle(.bordered)

            Text(sender.status).font(.footnote).foregroundStyle(.secondary)
            Text("Tracking: \(sender.tracking)").font(.footnote)
            Spacer()
        }
        .padding()
    }
}
