import SwiftUI
import ARKit
import Network
import UIKit

// Streams the iPhone camera pose (ARKit visual-inertial tracking) to Blender over UDP and
// lets you control Blender (record / recenter) from the phone.
//
//  phone -> Blender : AR1,seq,timestamp,px,py,pz,qx,qy,qz,qw,trackingState   (0 none, 1 limited, 2 normal)
//                     CMD,recenter | CMD,rec_start | CMD,rec_stop
//  Blender -> phone : ST,recording(0/1),samples,elapsedSeconds,takesSaved
//
// Threading: ARSession delivers frames on the main queue (no delegateQueue is set), the UDP
// callbacks hop back to the main queue, so every property below is only touched on main.

final class PoseSender: NSObject, ObservableObject, ARSessionDelegate {
    // UI state (only published when the value really changes)
    @Published var running = false
    @Published var trackingLevel = 0          // 0 none, 1 limited, 2 normal
    @Published var trackingText = "Not streaming"
    @Published var hint = ""
    @Published var online = false             // Blender answered in the last 1.5 s
    @Published var hz = 0
    @Published var recording = false
    @Published var samples = 0
    @Published var elapsed = 0.0
    @Published var takes = 0
    @Published var netProblem = ""
    @Published var hot = false

    var orientIndex = 0                       // 0 portrait, 1 top of phone to the right, 2 top to the left

    private let session = ARSession()
    private let udpQueue = DispatchQueue(label: "arcam.udp")
    private var connection: NWConnection?
    private var endpointHost: NWEndpoint.Host?
    private var endpointPort: NWEndpoint.Port?
    private var seq: Int32 = 0
    private var packetCount = 0
    private var lastCount = 0
    private var lastStatus = Date.distantPast
    private var statsTimer: Timer?
    private var receiving = false             // only one receive loop per connection
    private var pendingRecenter = false       // after "Reset tracking": re-anchor Blender automatically
    private var resetUptime: TimeInterval = 0 // frames older than this belong to the old AR world
    private var reconnectScheduled = false

    // MARK: start / stop

    func start(host: String, port: String, orientation: Int) {
        stop()
        guard !host.isEmpty, let p = UInt16(port), p > 0, let nwPort = NWEndpoint.Port(rawValue: p) else {
            netProblem = "Enter the computer's IP address and port in Settings."
            return
        }
        netProblem = ""
        orientIndex = orientation
        endpointHost = NWEndpoint.Host(host)
        endpointPort = nwPort
        openConnection()

        seq = 0
        packetCount = 0
        lastCount = 0
        lastStatus = .distantPast
        pendingRecenter = false
        beginAR(reset: true)
        running = true
        trackingText = "Starting…"
        UIApplication.shared.isIdleTimerDisabled = true      // keep the screen on

        statsTimer?.invalidate()
        statsTimer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in
            self?.updateStats()
        }
    }

    func stop() {
        statsTimer?.invalidate()
        statsTimer = nil
        session.pause()
        closeConnection()
        endpointHost = nil
        endpointPort = nil
        UIApplication.shared.isIdleTimerDisabled = false
        running = false
        online = false
        hz = 0
        recording = false
        samples = 0
        elapsed = 0
        takes = 0
        trackingLevel = 0
        trackingText = "Not streaming"
        hint = ""
        pendingRecenter = false
    }

    // MARK: UDP connection (re-created automatically if it fails)

    private func openConnection() {
        guard let host = endpointHost, let port = endpointPort else { return }
        let c = NWConnection(host: host, port: port, using: .udp)
        receiving = false
        c.stateUpdateHandler = { [weak self, weak c] state in
            DispatchQueue.main.async {
                guard let self = self, let c = c, self.connection === c else { return }
                switch state {
                case .ready:
                    self.netProblem = ""
                    if !self.receiving {
                        self.receiving = true
                        self.receiveLoop(c)
                    }
                case .waiting(let error):
                    self.netProblem = self.describe(error)
                case .failed(let error):
                    self.netProblem = self.describe(error)
                    self.scheduleReconnect()
                default:
                    break
                }
            }
        }
        connection = c
        c.start(queue: udpQueue)
    }

    private func closeConnection() {
        if let c = connection {
            c.stateUpdateHandler = nil
            c.cancel()
        }
        connection = nil
        receiving = false
    }

    private func scheduleReconnect() {
        guard running, !reconnectScheduled else { return }
        reconnectScheduled = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { [weak self] in
            guard let self = self else { return }
            self.reconnectScheduled = false
            guard self.running else { return }
            self.closeConnection()
            self.openConnection()
        }
    }

    private func describe(_ error: NWError) -> String {
        if case .posix(let code) = error, code == .ENETDOWN {
            // Known iOS behaviour: Local Network access denied, or a change not picked up yet.
            return "iOS blocks local network access. Settings → Privacy & Security → Local Network → "
                + "turn ARCam on. If it is already on, restart the iPhone (known iOS issue)."
        }
        return "Network: \(error.localizedDescription)"
    }

    // MARK: AR session

    private func beginAR(reset: Bool) {
        let cfg = ARWorldTrackingConfiguration()
        cfg.worldAlignment = .gravity                        // Y up, gravity aligned
        cfg.planeDetection = []                              // not needed: saves CPU / heat
        cfg.environmentTexturing = .none
        cfg.isLightEstimationEnabled = false
        // Make sure we run at 60 fps (more pose samples, smoother motion). Only switch
        // format if the default one is slower.
        if cfg.videoFormat.framesPerSecond < 60,
           let f = ARWorldTrackingConfiguration.supportedVideoFormats.first(where: { $0.framesPerSecond >= 60 }) {
            cfg.videoFormat = f
        }
        session.delegate = self
        let options: ARSession.RunOptions = reset ? [.resetTracking, .removeExistingAnchors] : []
        session.run(cfg, options: options)
    }

    func resetTracking() {
        guard running else { return }
        resetUptime = ProcessInfo.processInfo.systemUptime   // same clock as ARFrame.timestamp
        beginAR(reset: true)
        pendingRecenter = true      // the AR world origin moved: Blender must re-anchor, or the camera jumps
        haptic(.medium)
    }

    // MARK: commands to / status from Blender

    func sendCommand(_ name: String) {
        send("CMD," + name)
        haptic(.medium)
    }

    private func send(_ text: String) {
        connection?.send(content: text.data(using: .utf8), completion: .contentProcessed { _ in })
    }

    private func receiveLoop(_ c: NWConnection) {
        c.receiveMessage { [weak self, weak c] data, _, _, error in
            DispatchQueue.main.async {
                guard let self = self, let c = c, self.connection === c else { return }
                if let data = data, let text = String(data: data, encoding: .utf8) {
                    self.handleStatus(text)
                }
                if error == nil {
                    self.receiveLoop(c)
                } else {
                    self.receiving = false
                }
            }
        }
    }

    private func handleStatus(_ text: String) {
        let parts = text.trimmingCharacters(in: .whitespacesAndNewlines).split(separator: ",")
        guard parts.count >= 5, parts[0] == "ST",
              let rec = Int(parts[1]), let n = Int(parts[2]),
              let el = Double(parts[3]), let t = Int(parts[4]) else { return }
        lastStatus = Date()
        let isRec = rec == 1
        if isRec != recording { haptic(.heavy) }             // confirmation that Blender really changed state
        recording = isRec
        samples = n
        elapsed = el
        takes = t
        if !online { online = true }
    }

    private func updateStats() {
        let n = packetCount
        hz = Int((Double(n - lastCount) / 0.5).rounded())
        lastCount = n
        let isOnline = Date().timeIntervalSince(lastStatus) < 1.5
        if isOnline != online {
            online = isOnline
            if !isOnline { recording = false }
        }
        let isHot = ProcessInfo.processInfo.thermalState.rawValue >= ProcessInfo.ThermalState.serious.rawValue
        if isHot != hot { hot = isHot }
    }

    private func haptic(_ style: UIImpactFeedbackGenerator.FeedbackStyle) {
        UIImpactFeedbackGenerator(style: style).impactOccurred()
    }

    // MARK: ARSessionDelegate

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let cam = frame.camera

        // Tracking quality, in plain words (published only when it changes)
        var level = 0
        var text = "No tracking"
        var newHint = "Point the camera at the room and move gently"
        switch cam.trackingState {
        case .normal:
            level = 2; text = "Tracking good"; newHint = ""
        case .limited(let reason):
            level = 1; text = "Tracking limited"
            switch reason {
            case .excessiveMotion:       newHint = "Move more slowly"
            case .insufficientFeatures:  newHint = "Point at a textured, well-lit area"
            case .initializing:          newHint = "Initializing… move the phone gently"
            case .relocalizing:          newHint = "Recovering… go back to where you were"
            @unknown default:            newHint = ""
            }
        case .notAvailable:
            break
        }
        if level != trackingLevel || newHint != hint {
            if level < trackingLevel { haptic(.rigid) }      // tracking got worse
            trackingLevel = level
            trackingText = text
            hint = newHint
        }

        guard connection != nil else { return }
        let orientation: UIInterfaceOrientation =
            orientIndex == 1 ? .landscapeLeft : (orientIndex == 2 ? .landscapeRight : .portrait)
        let m = simd_inverse(cam.viewMatrix(for: orientation))   // camera-to-world, screen-oriented
        let p = m.columns.3
        let q = simd_quatf(m)

        seq &+= 1
        packetCount += 1
        let line = String(format: "AR1,%d,%.6f,%.5f,%.5f,%.5f,%.7f,%.7f,%.7f,%.7f,%d",
                          seq, frame.timestamp,
                          Double(p.x), Double(p.y), Double(p.z),
                          Double(q.imag.x), Double(q.imag.y), Double(q.imag.z), Double(q.real),
                          Int32(level))
        send(line)

        // First usable pose after "Reset tracking": tell Blender to re-anchor on it.
        if pendingRecenter && level >= 1 && frame.timestamp > resetUptime {
            pendingRecenter = false
            send("CMD,recenter")
        }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        trackingLevel = 0
        trackingText = "AR error"
        hint = error.localizedDescription
    }

    func sessionWasInterrupted(_ session: ARSession) {
        trackingLevel = 0
        trackingText = "Interrupted"
        hint = "Another app or a call took over the camera"
    }

    func sessionInterruptionEnded(_ session: ARSession) {
        hint = "Resuming… go back to where you were"
    }

    func sessionShouldAttemptRelocalization(_ session: ARSession) -> Bool { true }
}

// MARK: - UI

struct ContentView: View {
    @StateObject private var sender = PoseSender()
    @AppStorage("host") private var host = ""
    @AppStorage("port") private var port = "9000"
    @AppStorage("orient") private var orient = 0
    @State private var showSettings = false

    var body: some View {
        NavigationStack {
            VStack(spacing: 20) {
                statusCard
                Spacer(minLength: 0)
                if sender.running { controls }
                Spacer(minLength: 0)
                startButton
            }
            .padding()
            .navigationTitle("ARCam")
            .toolbar {
                ToolbarItem(placement: .primaryAction) {
                    Button { showSettings = true } label: { Image(systemName: "gearshape") }
                        .accessibilityLabel("Settings")
                }
            }
            .sheet(isPresented: $showSettings, onDismiss: { sender.orientIndex = orient }) {
                SettingsView(host: $host, port: $port, orient: $orient)
            }
            .onAppear { if host.isEmpty { showSettings = true } }
        }
    }

    // Status: tracking + connection in one glance
    private var statusCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            statusRow(color: trackingColor,
                      title: sender.trackingText,
                      subtitle: sender.running ? sender.hint : "Start streaming to begin")
            Divider()
            statusRow(color: connectionColor,
                      title: connectionTitle,
                      subtitle: connectionSubtitle)
            if sender.running && !sender.online && sender.hz > 0 && sender.netProblem.isEmpty {
                Label("No reply from Blender. Check the IP, that both devices share a network, the "
                      + "computer's firewall, and Settings → Privacy & Security → Local Network → ARCam.",
                      systemImage: "questionmark.circle")
                    .font(.footnote).foregroundStyle(.secondary)
            }
            if !sender.netProblem.isEmpty {
                Label(sender.netProblem, systemImage: "wifi.exclamationmark")
                    .font(.footnote).foregroundStyle(.orange)
            }
            if sender.hot {
                Label("The phone is getting hot: tracking may degrade", systemImage: "thermometer.high")
                    .font(.footnote).foregroundStyle(.orange)
            }
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.secondarySystemBackground), in: RoundedRectangle(cornerRadius: 16))
    }

    private func statusRow(color: Color, title: String, subtitle: String) -> some View {
        HStack(alignment: .top, spacing: 12) {
            Circle().fill(color).frame(width: 14, height: 14).padding(.top, 5)
            VStack(alignment: .leading, spacing: 2) {
                Text(title).font(.headline)
                if !subtitle.isEmpty {
                    Text(subtitle).font(.subheadline).foregroundStyle(.secondary)
                }
            }
        }
    }

    private var trackingColor: Color {
        guard sender.running else { return .gray }
        return sender.trackingLevel == 2 ? .green : (sender.trackingLevel == 1 ? .orange : .red)
    }
    private var connectionColor: Color {
        guard sender.running else { return .gray }
        return sender.online ? .green : .orange
    }
    private var connectionTitle: String {
        guard sender.running else { return "Blender" }
        return sender.online ? "Blender connected" : "Waiting for Blender…"
    }
    private var connectionSubtitle: String {
        let target = host.isEmpty ? "No address set" : "\(host):\(port)"
        return sender.running ? "\(sender.hz) Hz  →  \(target)" : target
    }

    // Record + recenter, only while streaming
    private var controls: some View {
        VStack(spacing: 20) {
            VStack(spacing: 4) {
                if sender.recording {
                    Text(timeString(sender.elapsed))
                        .font(.system(size: 48, weight: .semibold, design: .rounded))
                        .monospacedDigit().foregroundStyle(.red)
                    Text("\(sender.samples) samples").font(.footnote).foregroundStyle(.secondary)
                } else {
                    Text(sender.takes == 0 ? "Ready" : "\(sender.takes) take\(sender.takes == 1 ? "" : "s") saved")
                        .font(.title3).foregroundStyle(.secondary)
                }
            }

            Button {
                sender.sendCommand(sender.recording ? "rec_stop" : "rec_start")
            } label: {
                ZStack {
                    Circle().stroke(Color.primary.opacity(0.8), lineWidth: 4).frame(width: 88, height: 88)
                    RoundedRectangle(cornerRadius: sender.recording ? 8 : 34)
                        .fill(Color.red)
                        .frame(width: sender.recording ? 36 : 68, height: sender.recording ? 36 : 68)
                }
            }
            .buttonStyle(.plain)
            .disabled(!sender.online)
            .opacity(sender.online ? 1 : 0.35)
            .animation(.easeInOut(duration: 0.2), value: sender.recording)
            .accessibilityLabel(sender.recording ? "Stop recording" : "Start recording")

            HStack(spacing: 12) {
                Button { sender.sendCommand("recenter") } label: {
                    Label("Recenter", systemImage: "scope").frame(maxWidth: .infinity)
                }
                Button { sender.resetTracking() } label: {
                    Label("Reset tracking", systemImage: "arrow.counterclockwise").frame(maxWidth: .infinity)
                }
            }
            .buttonStyle(.bordered)
            .controlSize(.large)
            .disabled(sender.recording)
        }
    }

    private var startButton: some View {
        Button {
            if sender.running {
                sender.stop()
            } else {
                sender.start(host: host.trimmingCharacters(in: .whitespacesAndNewlines),
                             port: port.trimmingCharacters(in: .whitespacesAndNewlines),
                             orientation: orient)
            }
        } label: {
            Text(sender.running ? "Stop streaming" : "Start streaming").frame(maxWidth: .infinity)
        }
        .buttonStyle(.borderedProminent)
        .controlSize(.large)
        .tint(sender.running ? .gray : .green)
        .disabled(sender.recording)
    }

    private func timeString(_ t: Double) -> String {
        let s = max(0, Int(t))
        return String(format: "%d:%02d", s / 60, s % 60)
    }
}

struct SettingsView: View {
    @Binding var host: String
    @Binding var port: String
    @Binding var orient: Int
    @Environment(\.dismiss) private var dismiss

    private var version: String {
        (Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String) ?? "?"
    }

    var body: some View {
        NavigationStack {
            Form {
                Section {
                    TextField("IP address, e.g. 192.168.1.20", text: $host)
                        .keyboardType(.numbersAndPunctuation)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                    TextField("Port", text: $port)
                        .keyboardType(.numberPad)
                } header: {
                    Text("Computer running Blender")
                } footer: {
                    Text("The IP address is shown in the ARCam panel in Blender. Both devices must be on the same network. Changes apply the next time you start streaming.")
                }
                Section {
                    Picker("Orientation", selection: $orient) {
                        Text("Portrait").tag(0)
                        Text("Top right").tag(1)
                        Text("Top left").tag(2)
                    }
                    .pickerStyle(.segmented)
                } header: {
                    Text("How you hold the phone")
                } footer: {
                    Text("Landscape: choose where the TOP of the phone (the camera side) points while you look at the screen. If the view in Blender is rolled, switch this.")
                }
                Section {
                    LabeledContent("Version", value: version)
                }
            }
            .navigationTitle("Settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .confirmationAction) { Button("Done") { dismiss() } }
            }
        }
    }
}
