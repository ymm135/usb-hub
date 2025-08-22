# USB3.0 HUB 上位机软件设计文档

## 1. 软件架构概述

### 1.1 系统架构

本上位机软件采用分层架构设计，支持5路USB HUB管理，其中端口5专用于系统通信。

```
┌─────────────────────────────────────────────────────────┐
│                   用户界面层 (UI Layer)                   │
├─────────────────────────────────────────────────────────┤
│  主界面  │  设备管理  │  授权控制  │  日志查看  │  系统设置  │
├─────────────────────────────────────────────────────────┤
│                   业务逻辑层 (Business Layer)             │
├─────────────────────────────────────────────────────────┤
│  设备监控  │  授权管理  │  策略引擎  │  日志管理  │  配置管理  │
├─────────────────────────────────────────────────────────┤
│                   通信层 (Communication Layer)           │
├─────────────────────────────────────────────────────────┤
│  串口通信  │  协议解析  │  数据缓存  │  错误处理  │  重连机制  │
│  (通过端口5的CH340G实现与STM32通信)                        │
├─────────────────────────────────────────────────────────┤
│                   数据层 (Data Layer)                    │
├─────────────────────────────────────────────────────────┤
│  SQLite数据库  │  配置文件  │  日志文件  │  设备白名单  │  策略规则 │
└─────────────────────────────────────────────────────────┘
```

### 1.2 5路端口架构特点

- **端口1-4**: 用户可控端口，支持设备检测、授权控制、端口切换
- **端口5**: 系统保留端口，专用于CH340G串口通信，不参与用户设备管理
- **通信架构**: 通过端口5的CH340G实现与STM32的串口通信
- **界面设计**: 端口5在界面中标识为"系统端口"，状态只读显示

### 1.3 技术栈选择

- **开发语言**: C# (.NET 6.0)
- **UI框架**: WPF (Windows Presentation Foundation)
- **数据库**: SQLite
- **通信**: SerialPort
- **日志**: NLog
- **配置**: JSON配置文件
- **打包**: WiX Toolset

## 2. 项目结构

```
USBHubManager/
├── src/
│   ├── USBHubManager.Core/          # 核心业务逻辑
│   │   ├── Models/                  # 数据模型
│   │   ├── Services/                # 业务服务
│   │   ├── Communication/           # 通信模块
│   │   └── Database/                # 数据访问
│   ├── USBHubManager.UI/            # 用户界面
│   │   ├── Views/                   # 视图
│   │   ├── ViewModels/              # 视图模型
│   │   ├── Controls/                # 自定义控件
│   │   └── Resources/               # 资源文件
│   └── USBHubManager.Tests/         # 单元测试
├── docs/                            # 文档
├── setup/                           # 安装包
└── README.md
```

## 3. 数据模型设计

### 3.1 核心数据模型

```csharp
// Models/USBDevice.cs
public class USBDevice
{
    public int Id { get; set; }
    public string VID { get; set; }        // 厂商ID
    public string PID { get; set; }        // 产品ID
    public string Manufacturer { get; set; } // 制造商
    public string Product { get; set; }     // 产品名称
    public string SerialNumber { get; set; } // 序列号
    public DeviceType Type { get; set; }    // 设备类型
    public int PortNumber { get; set; }     // 端口号
    public DateTime ConnectTime { get; set; } // 连接时间
    public DateTime? DisconnectTime { get; set; } // 断开时间
    public AuthorizationStatus Status { get; set; } // 授权状态
    public string Remarks { get; set; }     // 备注
}

public enum DeviceType
{
    Unknown = 0,
    Keyboard = 1,
    Mouse = 2,
    Storage = 3,
    HIDOther = 4,
    Audio = 5,
    Video = 6,
    Prohibited = 7
}

public enum AuthorizationStatus
{
    Pending = 0,     // 等待授权
    Approved = 1,    // 已批准
    Denied = 2,      // 已拒绝
    Timeout = 3,     // 超时
    AutoApproved = 4 // 自动批准
}

// Models/PortStatus.cs
public class PortStatus
{
    public int PortNumber { get; set; }
    public bool IsEnabled { get; set; }
    public bool HasDevice { get; set; }
    public PortMode Mode { get; set; }
    public USBDevice CurrentDevice { get; set; }
    public DateTime LastActivity { get; set; }
    public bool IsSystemPort { get; set; }  // 是否为系统端口(端口5)
}

public enum PortMode
{
    MCU = 0,  // 连接到MCU
    PC = 1,   // 连接到PC
    System = 2 // 系统端口(端口5专用)
}

// Models/AuthorizationRule.cs
public class AuthorizationRule
{
    public int Id { get; set; }
    public string Name { get; set; }
    public RuleType Type { get; set; }
    public string Pattern { get; set; }     // 匹配模式
    public AuthorizationAction Action { get; set; }
    public bool IsEnabled { get; set; }
    public int Priority { get; set; }       // 优先级
    public DateTime CreateTime { get; set; }
    public string Description { get; set; }
}

public enum RuleType
{
    VIDPIDMatch = 1,      // VID:PID匹配
    ManufacturerMatch = 2, // 制造商匹配
    ProductMatch = 3,      // 产品名匹配
    DeviceTypeMatch = 4,   // 设备类型匹配
    SerialMatch = 5        // 序列号匹配
}

public enum AuthorizationAction
{
    Allow = 1,    // 允许
    Deny = 2,     // 拒绝
    Prompt = 3    // 提示用户
}
```

### 3.2 配置模型

```csharp
// Models/SystemConfiguration.cs
public class SystemConfiguration
{
    public CommunicationConfig Communication { get; set; }
    public AuthorizationConfig Authorization { get; set; }
    public SecurityConfig Security { get; set; }
    public UIConfig UI { get; set; }
    public LoggingConfig Logging { get; set; }
}

public class CommunicationConfig
{
    public string ComPort { get; set; } = "AUTO"; // 自动检测
    public int BaudRate { get; set; } = 115200;
    public int DataBits { get; set; } = 8;
    public Parity Parity { get; set; } = Parity.None;
    public StopBits StopBits { get; set; } = StopBits.One;
    public int TimeoutMs { get; set; } = 5000;
    public int RetryCount { get; set; } = 3;
    public int HeartbeatInterval { get; set; } = 30; // 秒
}

public class AuthorizationConfig
{
    public bool AutoApproveKeyboard { get; set; } = true;
    public bool AutoApproveMouse { get; set; } = true;
    public bool AutoApproveKnownDevices { get; set; } = false;
    public int AuthTimeoutSeconds { get; set; } = 30;
    public bool RequireAdminApproval { get; set; } = false;
    public bool EnableWhitelist { get; set; } = true;
    public bool EnableBlacklist { get; set; } = true;
}

public class SecurityConfig
{
    public bool LogAllDevices { get; set; } = true;
    public bool AlertOnProhibitedDevice { get; set; } = true;
    public bool AlertOnUnknownDevice { get; set; } = false;
    public bool EnableDeviceFingerprinting { get; set; } = true;
    public int MaxDevicesPerPort { get; set; } = 1;
    public bool BlockUnauthorizedDevices { get; set; } = true;
}
```

## 4. 通信模块设计

### 4.1 串口通信服务

```csharp
// Communication/SerialCommunicationService.cs
public class SerialCommunicationService : IDisposable
{
    private SerialPort _serialPort;
    private readonly object _lockObject = new object();
    private readonly Queue<byte[]> _sendQueue = new Queue<byte[]>();
    private readonly CancellationTokenSource _cancellationTokenSource;
    private Task _receiveTask;
    private Task _sendTask;
    
    public event EventHandler<DataReceivedEventArgs> DataReceived;
    public event EventHandler<ConnectionStatusEventArgs> ConnectionStatusChanged;
    
    public bool IsConnected => _serialPort?.IsOpen ?? false;
    
    public async Task<bool> ConnectAsync(string portName, int baudRate)
    {
        try
        {
            _serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = 1000,
                WriteTimeout = 1000
            };
            
            _serialPort.Open();
            
            // 启动接收和发送任务
            _receiveTask = Task.Run(ReceiveDataAsync, _cancellationTokenSource.Token);
            _sendTask = Task.Run(SendDataAsync, _cancellationTokenSource.Token);
            
            OnConnectionStatusChanged(true);
            return true;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to connect to serial port {PortName}", portName);
            return false;
        }
    }
    
    public void Disconnect()
    {
        try
        {
            _cancellationTokenSource.Cancel();
            _serialPort?.Close();
            OnConnectionStatusChanged(false);
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Error disconnecting from serial port");
        }
    }
    
    public void SendData(byte[] data)
    {
        lock (_lockObject)
        {
            _sendQueue.Enqueue(data);
        }
    }
    
    private async Task ReceiveDataAsync()
    {
        var buffer = new byte[1024];
        
        while (!_cancellationTokenSource.Token.IsCancellationRequested)
        {
            try
            {
                if (_serialPort.IsOpen && _serialPort.BytesToRead > 0)
                {
                    int bytesRead = _serialPort.Read(buffer, 0, buffer.Length);
                    if (bytesRead > 0)
                    {
                        var data = new byte[bytesRead];
                        Array.Copy(buffer, data, bytesRead);
                        OnDataReceived(data);
                    }
                }
                
                await Task.Delay(10, _cancellationTokenSource.Token);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                Logger.Error(ex, "Error receiving data from serial port");
                await Task.Delay(1000, _cancellationTokenSource.Token);
            }
        }
    }
    
    private async Task SendDataAsync()
    {
        while (!_cancellationTokenSource.Token.IsCancellationRequested)
        {
            try
            {
                byte[] dataToSend = null;
                
                lock (_lockObject)
                {
                    if (_sendQueue.Count > 0)
                    {
                        dataToSend = _sendQueue.Dequeue();
                    }
                }
                
                if (dataToSend != null && _serialPort.IsOpen)
                {
                    _serialPort.Write(dataToSend, 0, dataToSend.Length);
                }
                
                await Task.Delay(10, _cancellationTokenSource.Token);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                Logger.Error(ex, "Error sending data to serial port");
                await Task.Delay(1000, _cancellationTokenSource.Token);
            }
        }
    }
}
```

### 4.2 协议处理服务

```csharp
// Communication/ProtocolService.cs
public class ProtocolService
{
    private readonly SerialCommunicationService _communicationService;
    private readonly Dictionary<byte, TaskCompletionSource<ProtocolFrame>> _pendingRequests;
    private byte _sequenceNumber = 0;
    
    public event EventHandler<DeviceInfoEventArgs> DeviceInfoReceived;
    public event EventHandler<AuthRequestEventArgs> AuthorizationRequested;
    public event EventHandler<PortStatusEventArgs> PortStatusReceived;
    
    public ProtocolService(SerialCommunicationService communicationService)
    {
        _communicationService = communicationService;
        _communicationService.DataReceived += OnDataReceived;
        _pendingRequests = new Dictionary<byte, TaskCompletionSource<ProtocolFrame>>();
    }
    
    public async Task<bool> SendAuthorizationResponseAsync(byte portNumber, bool approved)
    {
        var frame = new ProtocolFrame
        {
            Command = CommandType.AuthResponse,
            Sequence = ++_sequenceNumber,
            Data = new byte[] { portNumber, (byte)(approved ? 1 : 0) }
        };
        
        return await SendFrameAsync(frame);
    }
    
    public async Task<PortStatus[]> QueryPortStatusAsync()
    {
        var frame = new ProtocolFrame
        {
            Command = CommandType.PortStatus,
            Sequence = ++_sequenceNumber,
            Data = new byte[0]
        };
        
        var response = await SendFrameWithResponseAsync(frame);
        if (response != null)
        {
            return ParsePortStatusResponse(response.Data);
        }
        
        return null;
    }
    
    public async Task<bool> ControlPortAsync(byte portNumber, PortControlAction action)
    {
        var frame = new ProtocolFrame
        {
            Command = CommandType.PortControl,
            Sequence = ++_sequenceNumber,
            Data = new byte[] { portNumber, (byte)action }
        };
        
        return await SendFrameAsync(frame);
    }
    
    private async Task<bool> SendFrameAsync(ProtocolFrame frame)
    {
        try
        {
            var data = frame.ToByteArray();
            _communicationService.SendData(data);
            return true;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to send protocol frame");
            return false;
        }
    }
    
    private async Task<ProtocolFrame> SendFrameWithResponseAsync(ProtocolFrame frame, int timeoutMs = 5000)
    {
        var tcs = new TaskCompletionSource<ProtocolFrame>();
        _pendingRequests[frame.Sequence] = tcs;
        
        try
        {
            await SendFrameAsync(frame);
            
            using (var cts = new CancellationTokenSource(timeoutMs))
            {
                cts.Token.Register(() => tcs.TrySetCanceled());
                return await tcs.Task;
            }
        }
        catch (OperationCanceledException)
        {
            Logger.Warning("Protocol frame response timeout for sequence {Sequence}", frame.Sequence);
            return null;
        }
        finally
        {
            _pendingRequests.Remove(frame.Sequence);
        }
    }
    
    private void OnDataReceived(object sender, DataReceivedEventArgs e)
    {
        try
        {
            var frames = ProtocolFrame.ParseFrames(e.Data);
            
            foreach (var frame in frames)
            {
                ProcessReceivedFrame(frame);
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Error processing received data");
        }
    }
    
    private void ProcessReceivedFrame(ProtocolFrame frame)
    {
        // 检查是否为响应帧
        if (_pendingRequests.TryGetValue(frame.Sequence, out var tcs))
        {
            tcs.SetResult(frame);
            return;
        }
        
        // 处理主动上报的帧
        switch (frame.Command)
        {
            case CommandType.DeviceInfo:
                ProcessDeviceInfo(frame);
                break;
                
            case CommandType.AuthRequest:
                ProcessAuthRequest(frame);
                break;
                
            case CommandType.PortStatus:
                ProcessPortStatus(frame);
                break;
                
            default:
                Logger.Warning("Received unknown command type: {Command}", frame.Command);
                break;
        }
    }
}
```

## 5. 业务服务设计

### 5.1 设备管理服务

```csharp
// Services/DeviceManagementService.cs
public class DeviceManagementService
{
    private readonly IDeviceRepository _deviceRepository;
    private readonly IAuthorizationService _authorizationService;
    private readonly ProtocolService _protocolService;
    private readonly ObservableCollection<USBDevice> _connectedDevices;
    
    public IReadOnlyObservableCollection<USBDevice> ConnectedDevices { get; }
    
    public event EventHandler<DeviceEventArgs> DeviceConnected;
    public event EventHandler<DeviceEventArgs> DeviceDisconnected;
    public event EventHandler<AuthorizationRequestEventArgs> AuthorizationRequired;
    
    public DeviceManagementService(
        IDeviceRepository deviceRepository,
        IAuthorizationService authorizationService,
        ProtocolService protocolService)
    {
        _deviceRepository = deviceRepository;
        _authorizationService = authorizationService;
        _protocolService = protocolService;
        _connectedDevices = new ObservableCollection<USBDevice>();
        ConnectedDevices = new ReadOnlyObservableCollection<USBDevice>(_connectedDevices);
        
        _protocolService.DeviceInfoReceived += OnDeviceInfoReceived;
        _protocolService.AuthorizationRequested += OnAuthorizationRequested;
    }
    
    private async void OnDeviceInfoReceived(object sender, DeviceInfoEventArgs e)
    {
        var device = new USBDevice
        {
            VID = e.DeviceInfo.VID,
            PID = e.DeviceInfo.PID,
            Manufacturer = e.DeviceInfo.Manufacturer,
            Product = e.DeviceInfo.Product,
            SerialNumber = e.DeviceInfo.SerialNumber,
            Type = e.DeviceInfo.Type,
            PortNumber = e.DeviceInfo.PortNumber,
            ConnectTime = DateTime.Now,
            Status = AuthorizationStatus.Pending
        };
        
        // 保存到数据库
        await _deviceRepository.AddAsync(device);
        
        // 添加到连接设备列表
        Application.Current.Dispatcher.Invoke(() => {
            _connectedDevices.Add(device);
        });
        
        // 触发设备连接事件
        DeviceConnected?.Invoke(this, new DeviceEventArgs(device));
        
        Logger.Info("Device connected: {Manufacturer} {Product} on port {Port}", 
                   device.Manufacturer, device.Product, device.PortNumber);
    }
    
    private async void OnAuthorizationRequested(object sender, AuthRequestEventArgs e)
    {
        var device = _connectedDevices.FirstOrDefault(d => d.PortNumber == e.PortNumber);
        if (device == null) return;
        
        // 检查授权规则
        var decision = await _authorizationService.EvaluateAuthorizationAsync(device);
        
        switch (decision.Action)
        {
            case AuthorizationAction.Allow:
                await ApproveDeviceAsync(device, decision.Reason);
                break;
                
            case AuthorizationAction.Deny:
                await DenyDeviceAsync(device, decision.Reason);
                break;
                
            case AuthorizationAction.Prompt:
                // 需要用户确认
                AuthorizationRequired?.Invoke(this, new AuthorizationRequestEventArgs(device));
                break;
        }
    }
    
    public async Task<bool> ApproveDeviceAsync(USBDevice device, string reason = null)
    {
        try
        {
            // 发送授权响应
            var success = await _protocolService.SendAuthorizationResponseAsync(
                (byte)device.PortNumber, true);
            
            if (success)
            {
                device.Status = AuthorizationStatus.Approved;
                device.Remarks = reason ?? "Manually approved";
                
                await _deviceRepository.UpdateAsync(device);
                
                Logger.Info("Device approved: {Manufacturer} {Product} on port {Port}", 
                           device.Manufacturer, device.Product, device.PortNumber);
                
                return true;
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to approve device {DeviceId}", device.Id);
        }
        
        return false;
    }
    
    public async Task<bool> DenyDeviceAsync(USBDevice device, string reason = null)
    {
        try
        {
            // 发送拒绝响应
            var success = await _protocolService.SendAuthorizationResponseAsync(
                (byte)device.PortNumber, false);
            
            if (success)
            {
                device.Status = AuthorizationStatus.Denied;
                device.Remarks = reason ?? "Manually denied";
                
                await _deviceRepository.UpdateAsync(device);
                
                Logger.Info("Device denied: {Manufacturer} {Product} on port {Port}", 
                           device.Manufacturer, device.Product, device.PortNumber);
                
                return true;
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to deny device {DeviceId}", device.Id);
        }
        
        return false;
    }
    
    public async Task<List<USBDevice>> GetDeviceHistoryAsync(int days = 30)
    {
        var startDate = DateTime.Now.AddDays(-days);
        return await _deviceRepository.GetDevicesByDateRangeAsync(startDate, DateTime.Now);
    }
    
    public async Task<bool> DisconnectDeviceAsync(int portNumber)
    {
        try
        {
            var success = await _protocolService.ControlPortAsync(
                (byte)portNumber, PortControlAction.Disable);
            
            if (success)
            {
                var device = _connectedDevices.FirstOrDefault(d => d.PortNumber == portNumber);
                if (device != null)
                {
                    device.DisconnectTime = DateTime.Now;
                    await _deviceRepository.UpdateAsync(device);
                    
                    Application.Current.Dispatcher.Invoke(() => {
                        _connectedDevices.Remove(device);
                    });
                    
                    DeviceDisconnected?.Invoke(this, new DeviceEventArgs(device));
                }
            }
            
            return success;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to disconnect device on port {Port}", portNumber);
            return false;
        }
    }
}
```

### 5.2 授权服务

```csharp
// Services/AuthorizationService.cs
public class AuthorizationService : IAuthorizationService
{
    private readonly IAuthorizationRuleRepository _ruleRepository;
    private readonly SystemConfiguration _configuration;
    private readonly List<AuthorizationRule> _cachedRules;
    
    public AuthorizationService(
        IAuthorizationRuleRepository ruleRepository,
        SystemConfiguration configuration)
    {
        _ruleRepository = ruleRepository;
        _configuration = configuration;
        _cachedRules = new List<AuthorizationRule>();
        
        LoadRulesAsync();
    }
    
    public async Task<AuthorizationDecision> EvaluateAuthorizationAsync(USBDevice device)
    {
        try
        {
            // 1. 检查设备类型自动批准规则
            if (ShouldAutoApprove(device))
            {
                return new AuthorizationDecision
                {
                    Action = AuthorizationAction.Allow,
                    Reason = "Auto-approved based on device type",
                    RuleId = null
                };
            }
            
            // 2. 检查黑名单规则
            var blacklistRule = await CheckBlacklistRulesAsync(device);
            if (blacklistRule != null)
            {
                return new AuthorizationDecision
                {
                    Action = AuthorizationAction.Deny,
                    Reason = $"Blocked by rule: {blacklistRule.Name}",
                    RuleId = blacklistRule.Id
                };
            }
            
            // 3. 检查白名单规则
            if (_configuration.Authorization.EnableWhitelist)
            {
                var whitelistRule = await CheckWhitelistRulesAsync(device);
                if (whitelistRule != null)
                {
                    return new AuthorizationDecision
                    {
                        Action = AuthorizationAction.Allow,
                        Reason = $"Approved by rule: {whitelistRule.Name}",
                        RuleId = whitelistRule.Id
                    };
                }
                else
                {
                    // 启用白名单但设备不在白名单中
                    return new AuthorizationDecision
                    {
                        Action = AuthorizationAction.Deny,
                        Reason = "Device not in whitelist",
                        RuleId = null
                    };
                }
            }
            
            // 4. 检查自定义规则
            var customRule = await CheckCustomRulesAsync(device);
            if (customRule != null)
            {
                return new AuthorizationDecision
                {
                    Action = customRule.Action,
                    Reason = $"Matched rule: {customRule.Name}",
                    RuleId = customRule.Id
                };
            }
            
            // 5. 默认策略
            if (_configuration.Authorization.RequireAdminApproval)
            {
                return new AuthorizationDecision
                {
                    Action = AuthorizationAction.Prompt,
                    Reason = "Admin approval required",
                    RuleId = null
                };
            }
            else
            {
                return new AuthorizationDecision
                {
                    Action = AuthorizationAction.Allow,
                    Reason = "Default allow policy",
                    RuleId = null
                };
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Error evaluating authorization for device {VID}:{PID}", 
                        device.VID, device.PID);
            
            // 出错时默认拒绝
            return new AuthorizationDecision
            {
                Action = AuthorizationAction.Deny,
                Reason = "Authorization evaluation failed",
                RuleId = null
            };
        }
    }
    
    private bool ShouldAutoApprove(USBDevice device)
    {
        return (device.Type == DeviceType.Keyboard && _configuration.Authorization.AutoApproveKeyboard) ||
               (device.Type == DeviceType.Mouse && _configuration.Authorization.AutoApproveMouse);
    }
    
    private async Task<AuthorizationRule> CheckBlacklistRulesAsync(USBDevice device)
    {
        var blacklistRules = _cachedRules
            .Where(r => r.IsEnabled && r.Action == AuthorizationAction.Deny)
            .OrderBy(r => r.Priority);
        
        foreach (var rule in blacklistRules)
        {
            if (await MatchesRuleAsync(device, rule))
            {
                return rule;
            }
        }
        
        return null;
    }
    
    private async Task<AuthorizationRule> CheckWhitelistRulesAsync(USBDevice device)
    {
        var whitelistRules = _cachedRules
            .Where(r => r.IsEnabled && r.Action == AuthorizationAction.Allow)
            .OrderBy(r => r.Priority);
        
        foreach (var rule in whitelistRules)
        {
            if (await MatchesRuleAsync(device, rule))
            {
                return rule;
            }
        }
        
        return null;
    }
    
    private async Task<bool> MatchesRuleAsync(USBDevice device, AuthorizationRule rule)
    {
        switch (rule.Type)
        {
            case RuleType.VIDPIDMatch:
                var vidPid = $"{device.VID}:{device.PID}";
                return string.Equals(vidPid, rule.Pattern, StringComparison.OrdinalIgnoreCase);
                
            case RuleType.ManufacturerMatch:
                return device.Manufacturer?.Contains(rule.Pattern, StringComparison.OrdinalIgnoreCase) ?? false;
                
            case RuleType.ProductMatch:
                return device.Product?.Contains(rule.Pattern, StringComparison.OrdinalIgnoreCase) ?? false;
                
            case RuleType.DeviceTypeMatch:
                return Enum.TryParse<DeviceType>(rule.Pattern, out var deviceType) && 
                       device.Type == deviceType;
                
            case RuleType.SerialMatch:
                return string.Equals(device.SerialNumber, rule.Pattern, StringComparison.OrdinalIgnoreCase);
                
            default:
                return false;
        }
    }
    
    public async Task<List<AuthorizationRule>> GetRulesAsync()
    {
        return await _ruleRepository.GetAllAsync();
    }
    
    public async Task<bool> AddRuleAsync(AuthorizationRule rule)
    {
        try
        {
            rule.CreateTime = DateTime.Now;
            await _ruleRepository.AddAsync(rule);
            await LoadRulesAsync();
            return true;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to add authorization rule");
            return false;
        }
    }
    
    public async Task<bool> UpdateRuleAsync(AuthorizationRule rule)
    {
        try
        {
            await _ruleRepository.UpdateAsync(rule);
            await LoadRulesAsync();
            return true;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to update authorization rule {RuleId}", rule.Id);
            return false;
        }
    }
    
    public async Task<bool> DeleteRuleAsync(int ruleId)
    {
        try
        {
            await _ruleRepository.DeleteAsync(ruleId);
            await LoadRulesAsync();
            return true;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to delete authorization rule {RuleId}", ruleId);
            return false;
        }
    }
    
    private async Task LoadRulesAsync()
    {
        try
        {
            var rules = await _ruleRepository.GetAllAsync();
            _cachedRules.Clear();
            _cachedRules.AddRange(rules);
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to load authorization rules");
        }
    }
}

public class AuthorizationDecision
{
    public AuthorizationAction Action { get; set; }
    public string Reason { get; set; }
    public int? RuleId { get; set; }
}
```

## 6. 用户界面设计

### 6.1 主窗口设计

```xml
<!-- Views/MainWindow.xaml -->
<Window x:Class="USBHubManager.UI.Views.MainWindow"
        xmlns="http://schemas.microsoft.com/winfx/2006/xaml/presentation"
        xmlns:x="http://schemas.microsoft.com/winfx/2006/xaml"
        xmlns:d="http://schemas.microsoft.com/expression/blend/2008"
        xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"
        mc:Ignorable="d"
        Title="USB Hub Manager" Height="800" Width="1200"
        WindowStartupLocation="CenterScreen">
    
    <Grid>
        <Grid.RowDefinitions>
            <RowDefinition Height="Auto"/>
            <RowDefinition Height="*"/>
            <RowDefinition Height="Auto"/>
        </Grid.RowDefinitions>
        
        <!-- 菜单栏 -->
        <Menu Grid.Row="0">
            <MenuItem Header="文件">
                <MenuItem Header="导出日志" Command="{Binding ExportLogCommand}"/>
                <MenuItem Header="导入配置" Command="{Binding ImportConfigCommand}"/>
                <MenuItem Header="导出配置" Command="{Binding ExportConfigCommand}"/>
                <Separator/>
                <MenuItem Header="退出" Command="{Binding ExitCommand}"/>
            </MenuItem>
            <MenuItem Header="设备">
                <MenuItem Header="刷新设备列表" Command="{Binding RefreshDevicesCommand}"/>
                <MenuItem Header="断开所有设备" Command="{Binding DisconnectAllCommand}"/>
            </MenuItem>
            <MenuItem Header="设置">
                <MenuItem Header="系统配置" Command="{Binding OpenSettingsCommand}"/>
                <MenuItem Header="授权规则" Command="{Binding OpenRulesCommand}"/>
            </MenuItem>
            <MenuItem Header="帮助">
                <MenuItem Header="用户手册" Command="{Binding OpenManualCommand}"/>
                <MenuItem Header="关于" Command="{Binding AboutCommand}"/>
            </MenuItem>
        </Menu>
        
        <!-- 主内容区 -->
        <TabControl Grid.Row="1" Margin="5">
            <!-- 设备监控标签页 -->
            <TabItem Header="设备监控">
                <Grid>
                    <Grid.RowDefinitions>
                        <RowDefinition Height="Auto"/>
                        <RowDefinition Height="*"/>
                    </Grid.RowDefinitions>
                    
                    <!-- 端口状态显示 -->
                    <GroupBox Grid.Row="0" Header="端口状态" Margin="5">
                        <UniformGrid Rows="1" Columns="4" Margin="10">
                            <Border x:Name="Port1Status" BorderBrush="Gray" BorderThickness="2" 
                                    Margin="5" CornerRadius="5">
                                <StackPanel Orientation="Vertical" HorizontalAlignment="Center" 
                                           VerticalAlignment="Center" Margin="10">
                                    <TextBlock Text="端口 1" FontWeight="Bold" HorizontalAlignment="Center"/>
                                    <Ellipse Width="20" Height="20" Margin="5" 
                                            Fill="{Binding Port1Status, Converter={StaticResource StatusToColorConverter}}"/>
                                    <TextBlock Text="{Binding Port1DeviceName}" FontSize="10" 
                                              HorizontalAlignment="Center" TextWrapping="Wrap"/>
                                    <Button Content="断开" Margin="5" Padding="10,2" 
                                           Command="{Binding DisconnectPortCommand}" 
                                           CommandParameter="1"/>
                                </StackPanel>
                            </Border>
                            
                            <!-- 端口2 -->
                            <Border BorderBrush="Gray" BorderThickness="1" CornerRadius="5" Margin="5">
                                <StackPanel Margin="10">
                                    <TextBlock Text="端口 2" FontWeight="Bold" HorizontalAlignment="Center"/>
                                    <Ellipse Width="20" Height="20" Margin="5" 
                                            Fill="{Binding Port2Status, Converter={StaticResource StatusToColorConverter}}"/>
                                    <TextBlock Text="{Binding Port2DeviceName}" FontSize="10" 
                                              HorizontalAlignment="Center" TextWrapping="Wrap"/>
                                    <Button Content="断开" Margin="5" Padding="10,2" 
                                           Command="{Binding DisconnectPortCommand}" 
                                           CommandParameter="2"/>
                                </StackPanel>
                            </Border>
                            
                            <!-- 端口3 -->
                            <Border BorderBrush="Gray" BorderThickness="1" CornerRadius="5" Margin="5">
                                <StackPanel Margin="10">
                                    <TextBlock Text="端口 3" FontWeight="Bold" HorizontalAlignment="Center"/>
                                    <Ellipse Width="20" Height="20" Margin="5" 
                                            Fill="{Binding Port3Status, Converter={StaticResource StatusToColorConverter}}"/>
                                    <TextBlock Text="{Binding Port3DeviceName}" FontSize="10" 
                                              HorizontalAlignment="Center" TextWrapping="Wrap"/>
                                    <Button Content="断开" Margin="5" Padding="10,2" 
                                           Command="{Binding DisconnectPortCommand}" 
                                           CommandParameter="3"/>
                                </StackPanel>
                            </Border>
                            
                            <!-- 端口4 -->
                            <Border BorderBrush="Gray" BorderThickness="1" CornerRadius="5" Margin="5">
                                <StackPanel Margin="10">
                                    <TextBlock Text="端口 4" FontWeight="Bold" HorizontalAlignment="Center"/>
                                    <Ellipse Width="20" Height="20" Margin="5" 
                                            Fill="{Binding Port4Status, Converter={StaticResource StatusToColorConverter}}"/>
                                    <TextBlock Text="{Binding Port4DeviceName}" FontSize="10" 
                                              HorizontalAlignment="Center" TextWrapping="Wrap"/>
                                    <Button Content="断开" Margin="5" Padding="10,2" 
                                           Command="{Binding DisconnectPortCommand}" 
                                           CommandParameter="4"/>
                                </StackPanel>
                            </Border>
                            
                            <!-- 端口5 (系统端口) -->
                            <Border BorderBrush="Orange" BorderThickness="2" CornerRadius="5" Margin="5">
                                <StackPanel Margin="10">
                                    <TextBlock Text="端口 5 (系统)" FontWeight="Bold" HorizontalAlignment="Center" Foreground="Orange"/>
                                    <Ellipse Width="20" Height="20" Margin="5" 
                                            Fill="{Binding Port5Status, Converter={StaticResource StatusToColorConverter}}"/>
                                    <TextBlock Text="CH340G串口" FontSize="10" 
                                              HorizontalAlignment="Center" TextWrapping="Wrap"/>
                                    <TextBlock Text="(系统保留)" FontSize="8" 
                                              HorizontalAlignment="Center" Foreground="Gray"/>
                                </StackPanel>
                            </Border>
                        </UniformGrid>
                    </GroupBox>
                    
                    <!-- 当前连接设备列表 -->
                    <GroupBox Grid.Row="1" Header="当前连接设备" Margin="5">
                        <DataGrid ItemsSource="{Binding ConnectedDevices}" 
                                 AutoGenerateColumns="False" 
                                 CanUserAddRows="False" 
                                 CanUserDeleteRows="False"
                                 SelectionMode="Single"
                                 SelectedItem="{Binding SelectedDevice}">
                            <DataGrid.Columns>
                                <DataGridTextColumn Header="端口" Binding="{Binding PortNumber}" Width="60"/>
                                <DataGridTextColumn Header="设备类型" Binding="{Binding Type}" Width="80"/>
                                <DataGridTextColumn Header="制造商" Binding="{Binding Manufacturer}" Width="150"/>
                                <DataGridTextColumn Header="产品名称" Binding="{Binding Product}" Width="200"/>
                                <DataGridTextColumn Header="VID:PID" Width="100">
                                    <DataGridTextColumn.Binding>
                                        <MultiBinding StringFormat="{}{0}:{1}">
                                            <Binding Path="VID"/>
                                            <Binding Path="PID"/>
                                        </MultiBinding>
                                    </DataGridTextColumn.Binding>
                                </DataGridTextColumn>
                                <DataGridTextColumn Header="连接时间" Binding="{Binding ConnectTime, StringFormat=HH:mm:ss}" Width="80"/>
                                <DataGridTextColumn Header="状态" Binding="{Binding Status}" Width="80"/>
                                <DataGridTemplateColumn Header="操作" Width="120">
                                    <DataGridTemplateColumn.CellTemplate>
                                        <DataTemplate>
                                            <StackPanel Orientation="Horizontal">
                                                <Button Content="批准" Margin="2" Padding="5,2" 
                                                       Command="{Binding DataContext.ApproveDeviceCommand, RelativeSource={RelativeSource AncestorType=Window}}" 
                                                       CommandParameter="{Binding}"
                                                       IsEnabled="{Binding Status, Converter={StaticResource StatusToBoolConverter}}"/>
                                                <Button Content="拒绝" Margin="2" Padding="5,2" 
                                                       Command="{Binding DataContext.DenyDeviceCommand, RelativeSource={RelativeSource AncestorType=Window}}" 
                                                       CommandParameter="{Binding}"
                                                       IsEnabled="{Binding Status, Converter={StaticResource StatusToBoolConverter}}"/>
                                            </StackPanel>
                                        </DataTemplate>
                                    </DataGridTemplateColumn.CellTemplate>
                                </DataGridTemplateColumn>
                            </DataGrid.Columns>
                        </DataGrid>
                    </GroupBox>
                </Grid>
            </TabItem>
            
            <!-- 设备历史标签页 -->
            <TabItem Header="设备历史">
                <Grid>
                    <Grid.RowDefinitions>
                        <RowDefinition Height="Auto"/>
                        <RowDefinition Height="*"/>
                    </Grid.RowDefinitions>
                    
                    <!-- 查询条件 -->
                    <GroupBox Grid.Row="0" Header="查询条件" Margin="5">
                        <StackPanel Orientation="Horizontal" Margin="10">
                            <TextBlock Text="时间范围:" VerticalAlignment="Center" Margin="5"/>
                            <DatePicker SelectedDate="{Binding HistoryStartDate}" Margin="5"/>
                            <TextBlock Text="至" VerticalAlignment="Center" Margin="5"/>
                            <DatePicker SelectedDate="{Binding HistoryEndDate}" Margin="5"/>
                            <TextBlock Text="设备类型:" VerticalAlignment="Center" Margin="15,5,5,5"/>
                            <ComboBox ItemsSource="{Binding DeviceTypes}" 
                                     SelectedItem="{Binding SelectedDeviceType}" 
                                     Width="100" Margin="5"/>
                            <Button Content="查询" Command="{Binding SearchHistoryCommand}" 
                                   Margin="15,5,5,5" Padding="15,5"/>
                            <Button Content="导出" Command="{Binding ExportHistoryCommand}" 
                                   Margin="5" Padding="15,5"/>
                        </StackPanel>
                    </GroupBox>
                    
                    <!-- 历史记录列表 -->
                    <DataGrid Grid.Row="1" ItemsSource="{Binding DeviceHistory}" 
                             AutoGenerateColumns="False" 
                             CanUserAddRows="False" 
                             CanUserDeleteRows="False"
                             Margin="5">
                        <DataGrid.Columns>
                            <DataGridTextColumn Header="连接时间" Binding="{Binding ConnectTime, StringFormat=yyyy-MM-dd HH:mm:ss}" Width="140"/>
                            <DataGridTextColumn Header="断开时间" Binding="{Binding DisconnectTime, StringFormat=yyyy-MM-dd HH:mm:ss}" Width="140"/>
                            <DataGridTextColumn Header="端口" Binding="{Binding PortNumber}" Width="60"/>
                            <DataGridTextColumn Header="设备类型" Binding="{Binding Type}" Width="80"/>
                            <DataGridTextColumn Header="制造商" Binding="{Binding Manufacturer}" Width="150"/>
                            <DataGridTextColumn Header="产品名称" Binding="{Binding Product}" Width="200"/>
                            <DataGridTextColumn Header="VID:PID" Width="100">
                                <DataGridTextColumn.Binding>
                                    <MultiBinding StringFormat="{}{0}:{1}">
                                        <Binding Path="VID"/>
                                        <Binding Path="PID"/>
                                    </MultiBinding>
                                </DataGridTextColumn.Binding>
                            </DataGridTextColumn>
                            <DataGridTextColumn Header="授权状态" Binding="{Binding Status}" Width="80"/>
                            <DataGridTextColumn Header="备注" Binding="{Binding Remarks}" Width="*"/>
                        </DataGrid.Columns>
                    </DataGrid>
                </Grid>
            </TabItem>
            
            <!-- 授权规则标签页 -->
            <TabItem Header="授权规则">
                <!-- 授权规则管理界面 -->
            </TabItem>
            
            <!-- 系统日志标签页 -->
            <TabItem Header="系统日志">
                <!-- 系统日志显示界面 -->
            </TabItem>
        </TabControl>
        
        <!-- 状态栏 -->
        <StatusBar Grid.Row="2">
            <StatusBarItem>
                <TextBlock Text="{Binding ConnectionStatus}"/>
            </StatusBarItem>
            <Separator/>
            <StatusBarItem>
                <TextBlock Text="{Binding DeviceCount, StringFormat=设备数量: {0}}"/>
            </StatusBarItem>
            <Separator/>
            <StatusBarItem>
                <TextBlock Text="{Binding LastUpdateTime, StringFormat=最后更新: {0:HH:mm:ss}}"/>
            </StatusBarItem>
        </StatusBar>
    </Grid>
</Window>
```

### 6.2 主窗口ViewModel

```csharp
// ViewModels/MainWindowViewModel.cs
public class MainWindowViewModel : ViewModelBase
{
    private readonly DeviceManagementService _deviceManagementService;
    private readonly AuthorizationService _authorizationService;
    private readonly ProtocolService _protocolService;
    private readonly SerialCommunicationService _communicationService;
    
    private ObservableCollection<USBDevice> _connectedDevices;
    private ObservableCollection<USBDevice> _deviceHistory;
    private USBDevice _selectedDevice;
    private string _connectionStatus;
    private DateTime _lastUpdateTime;
    
    public IReadOnlyObservableCollection<USBDevice> ConnectedDevices { get; }
    public ObservableCollection<USBDevice> DeviceHistory 
    { 
        get => _deviceHistory; 
        set => SetProperty(ref _deviceHistory, value); 
    }
    
    public USBDevice SelectedDevice 
    { 
        get => _selectedDevice; 
        set => SetProperty(ref _selectedDevice, value); 
    }
    
    public string ConnectionStatus 
    { 
        get => _connectionStatus; 
        set => SetProperty(ref _connectionStatus, value); 
    }
    
    public DateTime LastUpdateTime 
    { 
        get => _lastUpdateTime; 
        set => SetProperty(ref _lastUpdateTime, value); 
    }
    
    public int DeviceCount => ConnectedDevices?.Count ?? 0;
    
    // 端口状态属性
    public PortConnectionStatus Port1Status { get; set; }
    public PortConnectionStatus Port2Status { get; set; }
    public PortConnectionStatus Port3Status { get; set; }
    public PortConnectionStatus Port4Status { get; set; }
    public PortConnectionStatus Port5Status { get; set; }  // 系统端口状态
    
    public string Port1DeviceName { get; set; }
    public string Port2DeviceName { get; set; }
    public string Port3DeviceName { get; set; }
    public string Port4DeviceName { get; set; }
    public string Port5DeviceName { get; set; } = "CH340G串口";  // 系统端口固定名称
    
    // 命令
    public ICommand RefreshDevicesCommand { get; }
    public ICommand ApproveDeviceCommand { get; }
    public ICommand DenyDeviceCommand { get; }
    public ICommand DisconnectPortCommand { get; }
    public ICommand DisconnectAllCommand { get; }
    public ICommand SearchHistoryCommand { get; }
    public ICommand ExportHistoryCommand { get; }
    public ICommand OpenSettingsCommand { get; }
    public ICommand OpenRulesCommand { get; }
    public ICommand ExitCommand { get; }
    
    public MainWindowViewModel(
        DeviceManagementService deviceManagementService,
        AuthorizationService authorizationService,
        ProtocolService protocolService,
        SerialCommunicationService communicationService)
    {
        _deviceManagementService = deviceManagementService;
        _authorizationService = authorizationService;
        _protocolService = protocolService;
        _communicationService = communicationService;
        
        ConnectedDevices = _deviceManagementService.ConnectedDevices;
        _deviceHistory = new ObservableCollection<USBDevice>();
        
        // 初始化命令
        RefreshDevicesCommand = new AsyncRelayCommand(RefreshDevicesAsync);
        ApproveDeviceCommand = new AsyncRelayCommand<USBDevice>(ApproveDeviceAsync);
        DenyDeviceCommand = new AsyncRelayCommand<USBDevice>(DenyDeviceAsync);
        DisconnectPortCommand = new AsyncRelayCommand<int>(DisconnectPortAsync);
        DisconnectAllCommand = new AsyncRelayCommand(DisconnectAllAsync);
        SearchHistoryCommand = new AsyncRelayCommand(SearchHistoryAsync);
        ExportHistoryCommand = new AsyncRelayCommand(ExportHistoryAsync);
        OpenSettingsCommand = new RelayCommand(OpenSettings);
        OpenRulesCommand = new RelayCommand(OpenRules);
        ExitCommand = new RelayCommand(Exit);
        
        // 订阅事件
        _deviceManagementService.DeviceConnected += OnDeviceConnected;
        _deviceManagementService.DeviceDisconnected += OnDeviceDisconnected;
        _deviceManagementService.AuthorizationRequired += OnAuthorizationRequired;
        _communicationService.ConnectionStatusChanged += OnConnectionStatusChanged;
        
        // 初始化连接状态
        ConnectionStatus = "未连接";
        
        // 启动连接
        _ = Task.Run(InitializeConnectionAsync);
    }
    
    private async Task InitializeConnectionAsync()
    {
        try
        {
            // 自动检测串口
            var portName = await DetectSerialPortAsync();
            if (!string.IsNullOrEmpty(portName))
            {
                var connected = await _communicationService.ConnectAsync(portName, 115200);
                if (connected)
                {
                    ConnectionStatus = $"已连接 - {portName}";
                    await RefreshDevicesAsync();
                }
                else
                {
                    ConnectionStatus = "连接失败";
                }
            }
            else
            {
                ConnectionStatus = "未找到设备";
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to initialize connection");
            ConnectionStatus = "连接错误";
        }
    }
    
    private async Task<string> DetectSerialPortAsync()
    {
        var portNames = SerialPort.GetPortNames();
        
        foreach (var portName in portNames)
        {
            try
            {
                using (var testPort = new SerialPort(portName, 115200))
                {
                    testPort.Open();
                    testPort.Close();
                    
                    // 这里可以添加更复杂的设备识别逻辑
                    // 比如发送特定命令并检查响应
                    
                    return portName;
                }
            }
            catch
            {
                // 端口不可用，继续尝试下一个
                continue;
            }
        }
        
        return null;
    }
    
    private async Task RefreshDevicesAsync()
    {
        try
        {
            var portStatuses = await _protocolService.QueryPortStatusAsync();
            if (portStatuses != null)
            {
                UpdatePortStatuses(portStatuses);
                LastUpdateTime = DateTime.Now;
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to refresh devices");
            await ShowErrorMessageAsync("刷新设备列表失败", ex.Message);
        }
    }
    
    private void UpdatePortStatuses(PortStatus[] portStatuses)
    {
        foreach (var status in portStatuses)
        {
            switch (status.PortNumber)
            {
                case 1:
                    Port1Status = status.HasDevice ? PortConnectionStatus.Connected : PortConnectionStatus.Disconnected;
                    Port1DeviceName = status.CurrentDevice?.Product ?? "无设备";
                    break;
                case 2:
                    Port2Status = status.HasDevice ? PortConnectionStatus.Connected : PortConnectionStatus.Disconnected;
                    Port2DeviceName = status.CurrentDevice?.Product ?? "无设备";
                    break;
                case 3:
                    Port3Status = status.HasDevice ? PortConnectionStatus.Connected : PortConnectionStatus.Disconnected;
                    Port3DeviceName = status.CurrentDevice?.Product ?? "无设备";
                    break;
                case 4:
                    Port4Status = status.HasDevice ? PortConnectionStatus.Connected : PortConnectionStatus.Disconnected;
                    Port4DeviceName = status.CurrentDevice?.Product ?? "无设备";
                    break;
            }
        }
        
        OnPropertyChanged(nameof(Port1Status));
        OnPropertyChanged(nameof(Port2Status));
        OnPropertyChanged(nameof(Port3Status));
        OnPropertyChanged(nameof(Port4Status));
        OnPropertyChanged(nameof(Port1DeviceName));
        OnPropertyChanged(nameof(Port2DeviceName));
        OnPropertyChanged(nameof(Port3DeviceName));
        OnPropertyChanged(nameof(Port4DeviceName));
        OnPropertyChanged(nameof(DeviceCount));
    }
    
    private async Task ApproveDeviceAsync(USBDevice device)
    {
        if (device == null) return;
        
        try
        {
            var success = await _deviceManagementService.ApproveDeviceAsync(device, "手动批准");
            if (success)
            {
                await ShowInfoMessageAsync("设备已批准", $"设备 {device.Product} 已成功批准连接。");
            }
            else
            {
                await ShowErrorMessageAsync("批准失败", "无法批准设备连接，请检查设备状态。");
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to approve device {DeviceId}", device.Id);
            await ShowErrorMessageAsync("批准失败", ex.Message);
        }
    }
    
    private async Task DenyDeviceAsync(USBDevice device)
    {
        if (device == null) return;
        
        try
        {
            var success = await _deviceManagementService.DenyDeviceAsync(device, "手动拒绝");
            if (success)
            {
                await ShowInfoMessageAsync("设备已拒绝", $"设备 {device.Product} 已被拒绝连接。");
            }
            else
            {
                await ShowErrorMessageAsync("拒绝失败", "无法拒绝设备连接，请检查设备状态。");
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to deny device {DeviceId}", device.Id);
            await ShowErrorMessageAsync("拒绝失败", ex.Message);
        }
    }
    
    private void OnDeviceConnected(object sender, DeviceEventArgs e)
    {
        Application.Current.Dispatcher.Invoke(() => {
            LastUpdateTime = DateTime.Now;
            OnPropertyChanged(nameof(DeviceCount));
        });
    }
    
    private void OnDeviceDisconnected(object sender, DeviceEventArgs e)
    {
        Application.Current.Dispatcher.Invoke(() => {
            LastUpdateTime = DateTime.Now;
            OnPropertyChanged(nameof(DeviceCount));
        });
    }
    
    private async void OnAuthorizationRequired(object sender, AuthorizationRequestEventArgs e)
    {
        // 显示授权确认对话框
        var result = await ShowAuthorizationDialogAsync(e.Device);
        
        if (result.HasValue)
        {
            if (result.Value)
            {
                await _deviceManagementService.ApproveDeviceAsync(e.Device, "用户批准");
            }
            else
            {
                await _deviceManagementService.DenyDeviceAsync(e.Device, "用户拒绝");
            }
        }
    }
    
    private void OnConnectionStatusChanged(object sender, ConnectionStatusEventArgs e)
    {
        Application.Current.Dispatcher.Invoke(() => {
            ConnectionStatus = e.IsConnected ? "已连接" : "连接断开";
        });
    }
}

public enum PortConnectionStatus
{
    Disconnected,
    Connected,
    Error
}
```

## 7. 安装部署

### 7.1 WiX安装包配置

```xml
<!-- setup/Product.wxs -->
<?xml version="1.0" encoding="UTF-8"?>
<Wix xmlns="http://schemas.microsoft.com/wix/2006/wi">
  <Product Id="*" Name="USB Hub Manager" Language="1033" Version="1.0.0.0" 
           Manufacturer="Your Company" UpgradeCode="{12345678-1234-1234-1234-123456789012}">
    
    <Package InstallerVersion="200" Compressed="yes" InstallScope="perMachine" />
    
    <MajorUpgrade DowngradeErrorMessage="A newer version of [ProductName] is already installed." />
    
    <MediaTemplate EmbedCab="yes" />
    
    <Feature Id="ProductFeature" Title="USB Hub Manager" Level="1">
      <ComponentGroupRef Id="ProductComponents" />
      <ComponentRef Id="ApplicationShortcut" />
    </Feature>
    
    <!-- 安装目录 -->
    <Directory Id="TARGETDIR" Name="SourceDir">
      <Directory Id="ProgramFilesFolder">
        <Directory Id="INSTALLFOLDER" Name="USB Hub Manager" />
      </Directory>
      <Directory Id="ProgramMenuFolder">
        <Directory Id="ApplicationProgramsFolder" Name="USB Hub Manager"/>
      </Directory>
    </Directory>
    
    <!-- 应用程序组件 -->
    <ComponentGroup Id="ProductComponents" Directory="INSTALLFOLDER">
      <Component Id="MainExecutable">
        <File Id="USBHubManagerExe" Source="$(var.USBHubManager.UI.TargetPath)" KeyPath="yes" />
      </Component>
      <Component Id="CoreLibrary">
        <File Id="USBHubManagerCore" Source="$(var.USBHubManager.Core.TargetPath)" />
      </Component>
      <Component Id="Dependencies">
        <File Id="SystemDataSQLite" Source="$(var.USBHubManager.UI.TargetDir)System.Data.SQLite.dll" />
        <File Id="NLog" Source="$(var.USBHubManager.UI.TargetDir)NLog.dll" />
        <File Id="NewtonsoftJson" Source="$(var.USBHubManager.UI.TargetDir)Newtonsoft.Json.dll" />
      </Component>
      <Component Id="ConfigFiles">
        <File Id="AppConfig" Source="$(var.USBHubManager.UI.TargetDir)USBHubManager.UI.exe.config" />
        <File Id="NLogConfig" Source="$(var.USBHubManager.UI.TargetDir)NLog.config" />
      </Component>
    </ComponentGroup>
    
    <!-- 开始菜单快捷方式 -->
    <DirectoryRef Id="ApplicationProgramsFolder">
      <Component Id="ApplicationShortcut">
        <Shortcut Id="ApplicationStartMenuShortcut" 
                  Name="USB Hub Manager" 
                  Description="USB设备管理工具"
                  Target="[#USBHubManagerExe]" 
                  WorkingDirectory="INSTALLFOLDER" />
        <util:InternetShortcut Id="OnlineHelpShortcut"
                              Name="在线帮助"
                              Target="https://your-company.com/usb-hub-manager/help" />
        <RemoveFolder Id="ApplicationProgramsFolder" On="uninstall" />
        <RegistryValue Root="HKCU" Key="Software\Microsoft\USBHubManager" 
                      Name="installed" Type="integer" Value="1" KeyPath="yes" />
      </Component>
    </DirectoryRef>
    
    <!-- 注册表设置 -->
    <DirectoryRef Id="INSTALLFOLDER">
      <Component Id="RegistryEntries">
        <RegistryKey Root="HKLM" Key="Software\USBHubManager">
          <RegistryValue Name="InstallPath" Type="string" Value="[INSTALLFOLDER]" />
          <RegistryValue Name="Version" Type="string" Value="[ProductVersion]" />
        </RegistryKey>
      </Component>
    </DirectoryRef>
    
  </Product>
</Wix>
```

### 7.2 自动更新机制

```csharp
// Services/UpdateService.cs
public class UpdateService
{
    private readonly string _updateServerUrl;
    private readonly string _currentVersion;
    
    public event EventHandler<UpdateAvailableEventArgs> UpdateAvailable;
    
    public UpdateService(string updateServerUrl, string currentVersion)
    {
        _updateServerUrl = updateServerUrl;
        _currentVersion = currentVersion;
    }
    
    public async Task<UpdateInfo> CheckForUpdatesAsync()
    {
        try
        {
            using (var client = new HttpClient())
            {
                var response = await client.GetStringAsync($"{_updateServerUrl}/api/version");
                var updateInfo = JsonConvert.DeserializeObject<UpdateInfo>(response);
                
                if (IsNewerVersion(updateInfo.Version, _currentVersion))
                {
                    UpdateAvailable?.Invoke(this, new UpdateAvailableEventArgs(updateInfo));
                    return updateInfo;
                }
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to check for updates");
        }
        
        return null;
    }
    
    public async Task<bool> DownloadAndInstallUpdateAsync(UpdateInfo updateInfo)
    {
        try
        {
            var tempPath = Path.GetTempFileName();
            
            using (var client = new HttpClient())
            {
                var updateData = await client.GetByteArrayAsync(updateInfo.DownloadUrl);
                await File.WriteAllBytesAsync(tempPath, updateData);
            }
            
            // 启动安装程序
            Process.Start(new ProcessStartInfo
            {
                FileName = tempPath,
                Arguments = "/S", // 静默安装
                UseShellExecute = true
            });
            
            // 退出当前应用程序
            Application.Current.Shutdown();
            
            return true;
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Failed to download and install update");
            return false;
        }
    }
    
    private bool IsNewerVersion(string newVersion, string currentVersion)
    {
        var newVer = new Version(newVersion);
        var currentVer = new Version(currentVersion);
        return newVer > currentVer;
    }
}

public class UpdateInfo
{
    public string Version { get; set; }
    public string DownloadUrl { get; set; }
    public string ReleaseNotes { get; set; }
    public DateTime ReleaseDate { get; set; }
    public bool IsRequired { get; set; }
}
```

## 8. 测试策略

### 8.1 单元测试

```csharp
// Tests/Services/AuthorizationServiceTests.cs
[TestClass]
public class AuthorizationServiceTests
{
    private Mock<IAuthorizationRuleRepository> _mockRuleRepository;
    private SystemConfiguration _testConfiguration;
    private AuthorizationService _authorizationService;
    
    [TestInitialize]
    public void Setup()
    {
        _mockRuleRepository = new Mock<IAuthorizationRuleRepository>();
        _testConfiguration = new SystemConfiguration
        {
            Authorization = new AuthorizationConfig
            {
                AutoApproveKeyboard = true,
                AutoApproveMouse = true,
                EnableWhitelist = true,
                RequireAdminApproval = false
            }
        };
        
        _authorizationService = new AuthorizationService(
            _mockRuleRepository.Object, 
            _testConfiguration);
    }
    
    [TestMethod]
    public async Task EvaluateAuthorizationAsync_KeyboardDevice_ShouldAutoApprove()
    {
        // Arrange
        var device = new USBDevice
        {
            Type = DeviceType.Keyboard,
            VID = "046D",
            PID = "C52B",
            Manufacturer = "Logitech",
            Product = "USB Receiver"
        };
        
        // Act
        var decision = await _authorizationService.EvaluateAuthorizationAsync(device);
        
        // Assert
        Assert.AreEqual(AuthorizationAction.Allow, decision.Action);
        Assert.IsTrue(decision.Reason.Contains("Auto-approved"));
    }
    
    [TestMethod]
    public async Task EvaluateAuthorizationAsync_BlacklistedDevice_ShouldDeny()
    {
        // Arrange
        var device = new USBDevice
        {
            Type = DeviceType.Storage,
            VID = "1234",
            PID = "5678",
            Manufacturer = "BadUSB Inc",
            Product = "Malicious Device"
        };
        
        var blacklistRule = new AuthorizationRule
        {
            Id = 1,
            Name = "Block BadUSB",
            Type = RuleType.VIDPIDMatch,
            Pattern = "1234:5678",
            Action = AuthorizationAction.Deny,
            IsEnabled = true,
            Priority = 1
        };
        
        _mockRuleRepository.Setup(r => r.GetAllAsync())
            .ReturnsAsync(new List<AuthorizationRule> { blacklistRule });
        
        // Act
        var decision = await _authorizationService.EvaluateAuthorizationAsync(device);
        
        // Assert
        Assert.AreEqual(AuthorizationAction.Deny, decision.Action);
        Assert.AreEqual(blacklistRule.Id, decision.RuleId);
    }
}
```

### 8.2 集成测试

```csharp
// Tests/Integration/CommunicationIntegrationTests.cs
[TestClass]
public class CommunicationIntegrationTests
{
    private SerialCommunicationService _communicationService;
    private ProtocolService _protocolService;
    
    [TestInitialize]
    public void Setup()
    {
        _communicationService = new SerialCommunicationService();
        _protocolService = new ProtocolService(_communicationService);
    }
    
    [TestMethod]
    public async Task SendAuthorizationResponse_ValidPort_ShouldSucceed()
    {
        // 这个测试需要实际的硬件连接
        // 可以使用模拟器或测试设备
        
        // Arrange
        var connected = await _communicationService.ConnectAsync("COM3", 115200);
        Assert.IsTrue(connected, "Failed to connect to test device");
        
        // Act
        var result = await _protocolService.SendAuthorizationResponseAsync(1, true);
        
        // Assert
        Assert.IsTrue(result);
    }
    
    [TestCleanup]
    public void Cleanup()
    {
        _communicationService?.Disconnect();
        _communicationService?.Dispose();
    }
}
```

## 9. 性能优化

### 9.1 内存管理

```csharp
// Services/MemoryManagementService.cs
public class MemoryManagementService
{
    private readonly Timer _gcTimer;
    private readonly PerformanceCounter _memoryCounter;
    
    public MemoryManagementService()
    {
        _memoryCounter = new PerformanceCounter("Process", "Working Set", 
                                               Process.GetCurrentProcess().ProcessName);
        
        // 每5分钟检查一次内存使用情况
        _gcTimer = new Timer(CheckMemoryUsage, null, 
                            TimeSpan.FromMinutes(5), 
                            TimeSpan.FromMinutes(5));
    }
    
    private void CheckMemoryUsage(object state)
    {
        try
        {
            var memoryUsage = _memoryCounter.NextValue() / (1024 * 1024); // MB
            
            if (memoryUsage > 100) // 超过100MB时进行垃圾回收
            {
                Logger.Info("Memory usage: {MemoryUsage}MB, triggering GC", memoryUsage);
                
                GC.Collect();
                GC.WaitForPendingFinalizers();
                GC.Collect();
                
                var newMemoryUsage = _memoryCounter.NextValue() / (1024 * 1024);
                Logger.Info("Memory usage after GC: {MemoryUsage}MB", newMemoryUsage);
            }
        }
        catch (Exception ex)
        {
            Logger.Error(ex, "Error checking memory usage");
        }
    }
    
    public void Dispose()
    {
        _gcTimer?.Dispose();
        _memoryCounter?.Dispose();
    }
}
```

### 9.2 数据库优化

```csharp
// Database/OptimizedDeviceRepository.cs
public class OptimizedDeviceRepository : IDeviceRepository
{
    private readonly string _connectionString;
    private readonly SemaphoreSlim _connectionSemaphore;
    
    public OptimizedDeviceRepository(string connectionString)
    {
        _connectionString = connectionString;
        _connectionSemaphore = new SemaphoreSlim(10, 10); // 最多10个并发连接
    }
    
    public async Task<List<USBDevice>> GetDevicesByDateRangeAsync(DateTime startDate, DateTime endDate)
    {
        await _connectionSemaphore.WaitAsync();
        
        try
        {
            using (var connection = new SQLiteConnection(_connectionString))
            {
                await connection.OpenAsync();
                
                // 使用参数化查询和索引优化
                var sql = @"
                    SELECT * FROM USBDevices 
                    WHERE ConnectTime >= @StartDate AND ConnectTime <= @EndDate
                    ORDER BY ConnectTime DESC
                    LIMIT 1000";
                
                using (var command = new SQLiteCommand(sql, connection))
                {
                    command.Parameters.AddWithValue("@StartDate", startDate);
                    command.Parameters.AddWithValue("@EndDate", endDate);
                    
                    var devices = new List<USBDevice>();
                    
                    using (var reader = await command.ExecuteReaderAsync())
                    {
                        while (await reader.ReadAsync())
                        {
                            devices.Add(MapFromDataReader(reader));
                        }
                    }
                    
                    return devices;
                }
            }
        }
        finally
        {
            _connectionSemaphore.Release();
        }
    }
    
    // 批量插入优化
    public async Task<bool> AddDevicesBatchAsync(IEnumerable<USBDevice> devices)
    {
        await _connectionSemaphore.WaitAsync();
        
        try
        {
            using (var connection = new SQLiteConnection(_connectionString))
            {
                await connection.OpenAsync();
                
                using (var transaction = connection.BeginTransaction())
                {
                    try
                    {
                        var sql = @"
                            INSERT INTO USBDevices 
                            (VID, PID, Manufacturer, Product, SerialNumber, Type, PortNumber, ConnectTime, Status)
                            VALUES (@VID, @PID, @Manufacturer, @Product, @SerialNumber, @Type, @PortNumber, @ConnectTime, @Status)";
                        
                        using (var command = new SQLiteCommand(sql, connection, transaction))
                        {
                            foreach (var device in devices)
                            {
                                command.Parameters.Clear();
                                command.Parameters.AddWithValue("@VID", device.VID);
                                command.Parameters.AddWithValue("@PID", device.PID);
                                command.Parameters.AddWithValue("@Manufacturer", device.Manufacturer);
                                command.Parameters.AddWithValue("@Product", device.Product);
                                command.Parameters.AddWithValue("@SerialNumber", device.SerialNumber);
                                command.Parameters.AddWithValue("@Type", (int)device.Type);
                                command.Parameters.AddWithValue("@PortNumber", device.PortNumber);
                                command.Parameters.AddWithValue("@ConnectTime", device.ConnectTime);
                                command.Parameters.AddWithValue("@Status", (int)device.Status);
                                
                                await command.ExecuteNonQueryAsync();
                            }
                        }
                        
                        transaction.Commit();
                        return true;
                    }
                    catch
                    {
                        transaction.Rollback();
                        throw;
                    }
                }
            }
        }
        finally
        {
            _connectionSemaphore.Release();
        }
    }
}
```

## 10. 安全考虑

### 10.1 数据加密

```csharp
// Security/DataEncryption.cs
public class DataEncryption
{
    private readonly byte[] _key;
    private readonly byte[] _iv;
    
    public DataEncryption()
    {
        using (var aes = Aes.Create())
        {
            aes.GenerateKey();
            aes.GenerateIV();
            _key = aes.Key;
            _iv = aes.IV;
        }
    }
    
    public string EncryptString(string plainText)
    {
        if (string.IsNullOrEmpty(plainText))
            return plainText;
        
        using (var aes = Aes.Create())
        {
            aes.Key = _key;
            aes.IV = _iv;
            
            using (var encryptor = aes.CreateEncryptor())
            using (var msEncrypt = new MemoryStream())
            using (var csEncrypt = new CryptoStream(msEncrypt, encryptor, CryptoStreamMode.Write))
            using (var swEncrypt = new StreamWriter(csEncrypt))
            {
                swEncrypt.Write(plainText);
                swEncrypt.Close();
                return Convert.ToBase64String(msEncrypt.ToArray());
            }
        }
    }
    
    public string DecryptString(string cipherText)
    {
        if (string.IsNullOrEmpty(cipherText))
            return cipherText;
        
        using (var aes = Aes.Create())
        {
            aes.Key = _key;
            aes.IV = _iv;
            
            using (var decryptor = aes.CreateDecryptor())
            using (var msDecrypt = new MemoryStream(Convert.FromBase64String(cipherText)))
            using (var csDecrypt = new CryptoStream(msDecrypt, decryptor, CryptoStreamMode.Read))
            using (var srDecrypt = new StreamReader(csDecrypt))
            {
                return srDecrypt.ReadToEnd();
            }
        }
    }
}
```

## 11. 部署和维护

### 11.1 日志配置

```xml
<!-- NLog.config -->
<?xml version="1.0" encoding="utf-8" ?>
<nlog xmlns="http://www.nlog-project.org/schemas/NLog.xsd"
      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  
  <targets>
    <target xsi:type="File" name="fileTarget"
            fileName="${basedir}/logs/${shortdate}.log"
            layout="${longdate} ${level:uppercase=true} ${logger} ${message} ${exception:format=tostring}"
            archiveFileName="${basedir}/logs/archive/{#}.log"
            archiveEvery="Day"
            archiveNumbering="Rolling"
            maxArchiveFiles="30" />
    
    <target xsi:type="EventLog" name="eventLogTarget"
            source="USB Hub Manager"
            log="Application"
            layout="${message} ${exception:format=tostring}" />
  </targets>
  
  <rules>
    <logger name="*" minlevel="Info" writeTo="fileTarget" />
    <logger name="*" minlevel="Error" writeTo="eventLogTarget" />
  </rules>
</nlog>
```

### 11.2 配置文件模板

```json
// appsettings.json
{
  "Communication": {
    "ComPort": "AUTO",
    "BaudRate": 115200,
    "TimeoutMs": 5000,
    "RetryCount": 3,
    "HeartbeatInterval": 30
  },
  "Authorization": {
    "AutoApproveKeyboard": true,
    "AutoApproveMouse": true,
    "AutoApproveKnownDevices": false,
    "AuthTimeoutSeconds": 30,
    "RequireAdminApproval": false,
    "EnableWhitelist": true,
    "EnableBlacklist": true
  },
  "Security": {
    "LogAllDevices": true,
    "AlertOnProhibitedDevice": true,
    "AlertOnUnknownDevice": false,
    "EnableDeviceFingerprinting": true,
    "MaxDevicesPerPort": 1,
    "BlockUnauthorizedDevices": true
  },
  "UI": {
    "Theme": "Light",
    "Language": "zh-CN",
    "AutoRefreshInterval": 5,
    "ShowNotifications": true,
    "MinimizeToTray": true
  },
  "Logging": {
    "LogLevel": "Info",
    "MaxLogFiles": 30,
    "LogToEventLog": true,
    "LogToFile": true
  },
  "Database": {
    "ConnectionString": "Data Source=usb_hub_manager.db;Version=3;",
    "BackupInterval": 24,
    "MaxBackupFiles": 7
  }
}
```

## 12. 总结

本上位机软件设计采用了现代化的软件架构和技术栈，具有以下特点：

1. **模块化设计**: 清晰的分层架构，便于维护和扩展
2. **异步编程**: 使用async/await模式，提高响应性能
3. **MVVM模式**: 分离界面和业务逻辑，便于测试
4. **可配置性**: 丰富的配置选项，适应不同使用场景
5. **安全性**: 数据加密、权限控制、审计日志
6. **可维护性**: 完善的日志系统、错误处理、自动更新
7. **可扩展性**: 插件化架构，支持功能扩展

该软件能够完全满足USB3.0 HUB的管理需求，提供直观的用户界面和强大的设备控制功能。