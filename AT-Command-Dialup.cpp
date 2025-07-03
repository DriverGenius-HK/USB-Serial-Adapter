#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <memory>
#include <functional>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <sstream>

#ifdef _WIN32
    #include <windows.h>
    #include <conio.h>
#else
    #include <termios.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <termios.h>
    #include <unistd.h>
    #include <errno.h>
    #include <curses.h>
    #include <term.h>
#endif

class SerialPort {
private:
#ifdef _WIN32
    HANDLE hPort;
#else
    int fd;
#endif
    int baudrate;
    std::string portName;
    bool isOpen;

#ifdef _WIN32
    int getWinBaudrate(int baud) {
        switch (baud) {
            case 9600: return CBR_9600;
            case 19200: return CBR_19200;
            case 38400: return CBR_38400;
            case 57600: return CBR_57600;
            case 115200: return CBR_115200;
            case 128000: return CBR_128000;
            case 256000: return CBR_256000;
            default: return CBR_115200;
        }
    }
#endif

public:
    SerialPort() : baudrate(115200), isOpen(false) {
#ifdef _WIN32
        hPort = INVALID_HANDLE_VALUE;
#else
        fd = -1;
#endif
    }

    ~SerialPort() {
        close();
    }

    bool open(const std::string& port, int baud) {
        portName = port;
        baudrate = baud;
        isOpen = false;

#ifdef _WIN32
        std::string portStr = "\\\\.\\" + port;
        hPort = CreateFileA(
            portStr.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL
        );

        if (hPort == INVALID_HANDLE_VALUE) {
            std::cerr << "[ERROR] Failed to open port " << port << std::endl;
            return false;
        }

        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(hPort, &dcbSerialParams)) {
            std::cerr << "[ERROR] Failed to get port state" << std::endl;
            CloseHandle(hPort);
            return false;
        }

        dcbSerialParams.BaudRate = getWinBaudrate(baudrate);
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

        if (!SetCommState(hPort, &dcbSerialParams)) {
            std::cerr << "[ERROR] Failed to set port state" << std::endl;
            CloseHandle(hPort);
            return false;
        }

        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;

        if (!SetCommTimeouts(hPort, &timeouts)) {
            std::cerr << "[ERROR] Failed to set port timeouts" << std::endl;
            CloseHandle(hPort);
            return false;
        }

        isOpen = true;
#else
        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd == -1) {
            std::cerr << "[ERROR] Failed to open port " << port << ": " << strerror(errno) << std::endl;
            return false;
        }

        struct termios options;
        tcgetattr(fd, &options);

        cfsetispeed(&options, getBaudrate(baudrate));
        cfsetospeed(&options, getBaudrate(baudrate));

        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= CREAD | CLOCAL;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 5; // 0.5 seconds timeout

        tcsetattr(fd, TCSANOW, &options);

        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

        isOpen = true;
#endif

        return true;
    }

    void close() {
        if (isOpen) {
#ifdef _WIN32
            if (hPort != INVALID_HANDLE_VALUE) {
                CloseHandle(hPort);
                hPort = INVALID_HANDLE_VALUE;
            }
#else
            if (fd != -1) {
                ::close(fd);
                fd = -1;
            }
#endif
            isOpen = false;
        }
    }

    bool isOpenPort() const {
        return isOpen;
    }

    size_t write(const std::string& data) {
        if (!isOpen) return 0;

#ifdef _WIN32
        DWORD bytesWritten;
        if (!WriteFile(hPort, data.c_str(), data.length(), &bytesWritten, NULL)) {
            std::cerr << "[ERROR] Write error" << std::endl;
            return 0;
        }
        return bytesWritten;
#else
        return ::write(fd, data.c_str(), data.length());
#endif
    }

    std::string readAll() {
        if (!isOpen) return "";

        const int BUFFER_SIZE = 1024;
        char buffer[BUFFER_SIZE];
        std::string result;

#ifdef _WIN32
        DWORD bytesAvailable, bytesRead;
        if (!PeekNamedPipe(hPort, NULL, 0, NULL, &bytesAvailable, NULL) || bytesAvailable == 0) {
            return result;
        }

        if (bytesAvailable > BUFFER_SIZE) {
            bytesAvailable = BUFFER_SIZE;
        }

        if (!ReadFile(hPort, buffer, bytesAvailable, &bytesRead, NULL) || bytesRead == 0) {
            return result;
        }

        result = std::string(buffer, bytesRead);
#else
        int bytesAvailable = ::read(fd, buffer, BUFFER_SIZE);
        if (bytesAvailable > 0) {
            result = std::string(buffer, bytesAvailable);
        }
#endif

        return result;
    }

    std::string getPortName() const {
        return portName;
    }

private:
#ifndef _WIN32
    speed_t getBaudrate(int baud) {
        switch (baud) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            default: return B115200;
        }
    }
#endif
};

class SerialModemCLI {
public:
    static const char* VERSION;
    static const char* COMPANY;
    static const char* EMAIL;

private:
    SerialPort serialPort;
    std::atomic<bool> threadRunning;
    std::atomic<bool> isSending;
    std::queue<std::string> responseQueue;
    std::mutex queueMutex;
    std::condition_variable queueCV;
    std::vector<std::string> history;
    size_t historyPos;
    bool isWindows;
    std::string histFile;
    std::thread readThread;
    std::thread processThread;

    void _readFromPort() {
        std::string buffer;
        while (threadRunning && serialPort.isOpenPort()) {
            try {
                std::string data = serialPort.readAll();
                if (!data.empty()) {
                    buffer += data;
                    
                    size_t pos = 0;
                    while ((pos = buffer.find("\r\n")) != std::string::npos) {
                        std::string line = buffer.substr(0, pos);
                        buffer = buffer.substr(pos + 2);
                        
                        {
                            std::lock_guard<std::mutex> lock(queueMutex);
                            responseQueue.push(line);
                        }
                        queueCV.notify_one();
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            } catch (...) {
                threadRunning = false;
                break;
            }
        }
    }

    void _processResponses() {
        while (threadRunning) {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (responseQueue.empty()) {
                if (queueCV.wait_for(lock, std::chrono::milliseconds(100)) == std::cv_status::timeout) {
                    continue;
                }
            }

            if (!responseQueue.empty()) {
                std::string response = responseQueue.front();
                responseQueue.pop();
                lock.unlock();

                _printResponse(response);
            } else {
                lock.unlock();
            }
        }
    }

    void _resetModem() {
        send("ATZ");
        send("AT&F");
        send("ATE0");
    }

    void _printResponse(const std::string& response) {
        std::cout << "[RECV] " << response << std::endl;
        if (response.find("NO CARRIER") != std::string::npos) {
            std::cout << "[INFO] Connection lost" << std::endl;
        } else if (response.find("CONNECT") != std::string::npos) {
            std::cout << "[INFO] Connection established" << std::endl;
        } else if (response.find("RING") != std::string::npos) {
            std::cout << "[INFO] Ring detected" << std::endl;
        }
    }

#ifdef _WIN32
    std::string _getInputWindows() {
        std::vector<char> line;
        size_t histIdx = history.size();
        
        std::cout << "AT> ";
        std::cout.flush();
        
        while (true) {
            int key = _getch();
            if (key == '\r') {
                std::cout << std::endl;
                return std::string(line.begin(), line.end());
            } else if (key == '\b' || key == 8) { // Backspace
                if (!line.empty()) {
                    line.pop_back();
                    std::cout << "\b \b";
                    std::cout.flush();
                }
            } else if (key == 0 || key == 0xE0) { // Special keys (arrow keys)
                int arrow = _getch();
                if (arrow == 72 && histIdx > 0) { // Up arrow
                    histIdx--;
                    if (histIdx < history.size()) {
                        line.clear();
                        line = std::vector<char>(history[histIdx].begin(), history[histIdx].end());
                        std::cout << "\r" << std::string(4 + line.size() + 1, ' ') << "\rAT> ";
                        for (char c : line) {
                            std::cout << c;
                        }
                        std::cout.flush();
                    }
                } else if (arrow == 80 && histIdx < history.size()) { // Down arrow
                    histIdx++;
                    if (histIdx < history.size()) {
                        line.clear();
                        line = std::vector<char>(history[histIdx].begin(), history[histIdx].end());
                        std::cout << "\r" << std::string(4 + line.size() + 1, ' ') << "\rAT> ";
                        for (char c : line) {
                            std::cout << c;
                        }
                        std::cout.flush();
                    } else if (histIdx == history.size()) {
                        line.clear();
                        std::cout << "\r" << std::string(4 + 1, ' ') << "\rAT> ";
                        std::cout.flush();
                    }
                }
            } else if (key >= 32 && key <= 126) { // Printable characters
                line.push_back(static_cast<char>(key));
                std::cout << static_cast<char>(key);
                std::cout.flush();
            }
        }
    }
#endif

public:
    SerialModemCLI(const std::string& port = "", int baudrate = 115200)
        : threadRunning(false), isSending(false), historyPos(0), isWindows(false) {
        
        isWindows = (std::string(platform()) == "Windows");
        histFile = getHomeDir() + "/.at_cli_history";
        loadHistory();
        
        if (!port.empty()) {
            serialPort.open(port, baudrate);
        }
    }

    ~SerialModemCLI() {
        disconnect();
    }

    std::string platform() {
#ifdef _WIN32
        return "Windows";
#elif defined(__linux__)
        return "Linux";
#elif defined(__APPLE__)
        return "MacOS";
#else
        return "Unknown";
#endif
    }

    std::string getHomeDir() {
#ifdef _WIN32
        char path[MAX_PATH];
        if (GetEnvironmentVariableA("USERPROFILE", path, MAX_PATH)) {
            return path;
        }
        return "";
#else
        const char* home = getenv("HOME");
        return home ? home : "";
#endif
    }

    void loadHistory() {
        try {
            if (!histFile.empty() && std::ifstream(histFile).good()) {
                std::ifstream file(histFile);
                std::string line;
                history.clear();
                while (std::getline(file, line)) {
                    if (!line.empty()) {
                        history.push_back(line);
                    }
                }
            }
        } catch (...) {
            // Ignore errors
        }
    }

    void saveHistory() {
        try {
            std::ofstream file(histFile);
            for (const auto& cmd : history) {
                file << cmd << std::endl;
            }
        } catch (...) {
            // Ignore errors
        }
    }

    bool connect() {
        if (serialPort.isOpenPort()) {
            std::cout << "[INFO] Already connected" << std::endl;
            return true;
        }
        
        std::string port;
        if (serialPort.getPortName().empty()) {
            std::cout << "[ERROR] No port specified" << std::endl;
            return false;
        } else {
            port = serialPort.getPortName();
        }
        
        try {
            if (!serialPort.open(port, 115200)) {
                std::cout << "[ERROR] Failed to open port " << port << std::endl;
                return false;
            }
            
            std::cout << "[INFO] Connected to " << port << ", " << 115200 << "bps" << std::endl;
            _resetModem();
            _startThreads();
            return true;
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Connection failed: " << e.what() << std::endl;
            return false;
        }
    }

    void disconnect() {
        threadRunning = false;
        if (readThread.joinable()) {
            readThread.join();
        }
        if (processThread.joinable()) {
            processThread.join();
        }
        serialPort.close();
        saveHistory();
        std::cout << "[INFO] Disconnected" << std::endl;
    }

    void _startThreads() {
        threadRunning = true;
        readThread = std::thread(&SerialModemCLI::_readFromPort, this);
        processThread = std::thread(&SerialModemCLI::_processResponses, this);
    }

    void send(const std::string& cmd) {
        if (cmd.empty() || isSending) {
            return;
        }
        
        std::string trimmedCmd = trim(cmd);
        if (!trimmedCmd.empty() && (history.empty() || trimmedCmd != history.back())) {
            history.push_back(trimmedCmd);
            historyPos = history.size();
        }
        
        isSending = true;
        std::cout << "[SEND] " << trimmedCmd << std::endl;
        
        try {
            if (serialPort.isOpenPort()) {
                std::string cmdWithCrLf = trimmedCmd + "\r\n";
                serialPort.write(cmdWithCrLf);
            }
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Send error: " << e.what() << std::endl;
        }
        isSending = false;
    }

    std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(' ');
        if (first == std::string::npos) {
            return "";
        }
        size_t last = str.find_last_not_of(' ');
        return str.substr(first, (last - first + 1));
    }

    void showHelp() {
        std::cout << "\n=== AT Command Terminal v" << VERSION << " ===" << std::endl;
        std::cout << "(c) 2025 " << COMPANY << " | Email: " << EMAIL << std::endl << std::endl;
        std::cout << "Usage: at_cli.exe --port COM3 --baudrate 115200" << std::endl << std::endl;
        std::cout << "Commands:" << std::endl;
        std::cout << "  AT          - Test connection" << std::endl;
        std::cout << "  ATZ         - Reset modem" << std::endl;
        std::cout << "  AT+CSQ      - Check signal quality" << std::endl;
        std::cout << "  ATD<num>;   - Dial number (e.g. ATD10086;)" << std::endl;
        std::cout << "  ATH         - Hang up" << std::endl;
        std::cout << "  AT+CMGF=1   - Set SMS to text mode" << std::endl;
        std::cout << "  AT+CMGS=\"13800138000\" - Send SMS (followed by Ctrl+Z to send)" << std::endl;
        std::cout << "  help        - Show this help" << std::endl;
        std::cout << "  quit        - Exit" << std::endl;
        std::cout << "  list        - List available ports" << std::endl << std::endl;
        std::cout << "History: Use up/down arrows to navigate command history" << std::endl;
    }

    void listPorts() {
        try {
#ifdef _WIN32
            std::cout << "[INFO] Available ports:" << std::endl;
            for (int i = 0; i < 256; i++) {
                char portName[80];
                std::string portStr = "COM" + std::to_string(i + 1);
                DWORD result = QueryDosDeviceA(portStr.c_str(), portName, sizeof(portName));
                if (result != 0) {
                    std::cout << "  " << portStr << " - N/A" << std::endl;
                }
            }
#else
            std::cout << "[INFO] Available ports:" << std::endl;
            // On Unix-like systems, we can list ports in /dev
            std::system("ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | while read port; do echo \"  $port - N/A\"; done");
#endif
        } catch (const std::exception& e) {
            std::cout << "[ERROR] Port scan error: " << e.what() << std::endl;
        }
    }

    std::string getInput() {
        std::string prompt = "AT> ";
#ifdef _WIN32
        try {
            return _getInputWindows();
        } catch (...) {
            // Fallback to standard input
        }
#endif
        std::cout << prompt;
        std::string input;
        std::getline(std::cin, input);
        return input;
    }

    void run() {
        std::cout << "\n=== AT Command Terminal v" << VERSION << " ===" << std::endl;
        std::cout << "(c) 2025 " << COMPANY << " | Email: " << EMAIL << std::endl;
        std::cout << "Type 'help' for commands, 'quit' to exit" << std::endl;
        
        if (serialPort.getPortName().empty()) {
            listPorts();
            std::cout << "Enter serial port: ";
            std::string port;
            std::getline(std::cin, port);
            port = trim(port);
            if (port.empty()) {
                std::cout << "[ERROR] No port specified. Exiting." << std::endl;
                return;
            }
            serialPort.open(port, 115200);
        }
        
        if (!connect()) {
            return;
        }
        
        try {
            while (true) {
                std::string cmd = getInput();
                cmd = trim(cmd);
                if (cmd.empty()) {
                    continue;
                }
                
                if (cmd == "quit") {
                    break;
                } else if (cmd == "help") {
                    showHelp();
                } else if (cmd == "list") {
                    listPorts();
                } else {
                    send(cmd);
                }
            }
        } catch (const std::exception& e) {
            std::cout << "\n[INFO] Exiting... " << e.what() << std::endl;
        } catch (...) {
            std::cout << "\n[INFO] Exiting..." << std::endl;
        }
        disconnect();
    }
};

const char* SerialModemCLI::VERSION = "1.3.1";
const char* SerialModemCLI::COMPANY = "flykanTech.com";
const char* SerialModemCLI::EMAIL = "support@flykanTech.com";

int main(int argc, char* argv[]) {
    std::string port;
    int baudrate = 115200;
    
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "-p" || std::string(argv[i]) == "--port") {
            if (i + 1 < argc) {
                port = argv[i + 1];
                i++;
            }
        } else if (std::string(argv[i]) == "-b" || std::string(argv[i]) == "--baudrate") {
            if (i + 1 < argc) {
                baudrate = std::stoi(argv[i + 1]);
                i++;
            }
        }
    }
    
    std::cout << "FlykanTech AT Command Terminal v" << SerialModemCLI::VERSION << std::endl;
    std::cout << "(c) 2025 " << SerialModemCLI::COMPANY << " | Email: " << SerialModemCLI::EMAIL << std::endl;
    
    SerialModemCLI cli(port, baudrate);
    cli.run();
    
    return 0;
}