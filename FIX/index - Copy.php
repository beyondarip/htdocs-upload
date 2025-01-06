<?php
session_start();

// Configuration 
$uploadDir = 'uploads/';
$chatFile = 'chat.txt';

// Create uploads directory if it doesn't exist
if (!file_exists($uploadDir)) {
    mkdir($uploadDir, 0755, true);
}

// Create chat file if it doesn't exist
if (!file_exists($chatFile)) {
    file_put_contents($chatFile, '');
}

// Handle chat message submission via AJAX
if ($_SERVER['REQUEST_METHOD'] === 'POST' && isset($_POST['message'])) {
    $message = trim($_POST['message']);
    $timestamp = date('Y-m-d H:i:s');
    $formattedMessage = "$timestamp: $message\n---\n";  // Using --- as separator
    file_put_contents($chatFile, $formattedMessage, FILE_APPEND);
    if(isset($_POST['ajax'])) {
        exit;
    }
    header('Location: index.php');
    exit;
}

// If AJAX request for getting messages
if(isset($_GET['get_messages'])) {
    $messages = file_exists($chatFile) ? file_get_contents($chatFile) : '';
    $messageLines = array_filter(explode("\n---\n", $messages));  // Split by --- separator
    foreach ($messageLines as $line) {
        $line = trim($line);
        if (empty($line)) continue;
        
        // Separate timestamp and message content
        $parts = explode(': ', $line, 2);
        $timestamp = $parts[0];
        $messageContent = isset($parts[1]) ? $parts[1] : '';
        
        echo '<div class="message-line">
                <div class="message-content">
                    <span class="timestamp">' . htmlspecialchars($timestamp) . ': </span>
                    <span class="text">' . nl2br(htmlspecialchars($messageContent)) . '</span>
                </div>
                <button class="copy-btn" title="Copy message">
                    <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                        <rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect>
                        <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path>
                    </svg>
                </button>
              </div>';
    }
    exit;
}

// Handle file download
if (isset($_GET['download'])) {
    $filename = basename($_GET['download']);
    $filepath = $uploadDir . $filename;
    
    if (file_exists($filepath) && is_file($filepath)) {
        header('Content-Description: File Transfer');
        header('Content-Type: application/octet-stream');
        header('Content-Disposition: attachment; filename="' . $filename . '"');
        header('Content-Length: ' . filesize($filepath));
        header('Pragma: public');
        
        readfile($filepath);
        exit;
    }
}

// Get all files in upload directory
$files = array_diff(scandir($uploadDir), array('.', '..'));

// Read chat messages
$messages = file_exists($chatFile) ? file_get_contents($chatFile) : '';
$messageLines = array_filter(explode("\n", $messages));
?>

<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>File Management and Chat System</title>
    <style>
        :root {
            --bg-primary: #1a1a1a;
            --bg-secondary: #2d2d2d;
            --text-primary: #ffffff;
            --text-secondary: #b3b3b3;
            --accent-color: #007bff;
            --accent-hover: #0056b3;
            --border-color: #404040;
        }

        body {
            font-family: Arial, sans-serif;
            max-width: 1200px;
            margin: 20px auto;
            padding: 0 20px;
            background-color: var(--bg-primary);
            color: var(--text-primary);
        }

        .container {
            display: flex;
            gap: 20px;
        }

        .file-section, .chat-section {
            flex: 1;
            background: var(--bg-secondary);
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
        }

        .file-list {
            margin-top: 20px;
        }

        .file-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 12px;
            border-bottom: 1px solid var(--border-color);
            transition: background-color 0.2s;
        }

        .file-item:hover {
            background-color: rgba(255, 255, 255, 0.05);
        }

        .upload-form {
            margin-bottom: 20px;
        }

        input[type="file"] {
            background: var(--bg-primary);
            color: var(--text-primary);
            padding: 8px;
            border: 1px solid var(--border-color);
            border-radius: 4px;
            margin-right: 10px;
        }

        input[type="submit"], button {
            padding: 8px 16px;
            background: var(--accent-color);
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            transition: background-color 0.2s;
        }

        input[type="submit"]:hover, button:hover {
            background: var(--accent-hover);
        }

        .success {
            color: #4caf50;
            margin: 10px 0;
        }

        .error {
            color: #f44336;
            margin: 10px 0;
        }

        .chat-messages {
            height: 400px;
            overflow-y: auto;
            border: 1px solid var(--border-color);
            padding: 15px;
            margin-bottom: 20px;
            background: var(--bg-primary);
            border-radius: 4px;
        }

        .message-line {
            margin-bottom: 10px;
            padding: 10px;
            background: var(--bg-secondary);
            border-radius: 6px;
            display: flex;
            justify-content: space-between;
            align-items: flex-start;
        }

        .message-content {
            flex: 1;
            margin-right: 10px;
            white-space: pre-wrap;
        }

        .copy-btn {
            background: transparent;
            border: none;
            color: var(--text-secondary);
            cursor: pointer;
            padding: 4px;
            opacity: 0.7;
            transition: opacity 0.2s;
        }

        .copy-btn:hover {
            opacity: 1;
            background: transparent;
        }

        .chat-form {
            display: flex;
            gap: 10px;
        }

        .chat-form textarea {
            flex: 1;
            padding: 12px;
            background: var(--bg-primary);
            color: var(--text-primary);
            border: 1px solid var(--border-color);
            border-radius: 4px;
            resize: vertical;
            min-height: 60px;
        }

        .chat-form button {
            align-self: flex-end;
        }

        a {
            color: var(--accent-color);
            text-decoration: none;
            transition: color 0.2s;
        }

        a:hover {
            color: var(--accent-hover);
        }

        h1, h2 {
            color: var(--text-primary);
            border-bottom: 2px solid var(--border-color);
            padding-bottom: 8px;
        }

        /* Scrollbar Styling */
        ::-webkit-scrollbar {
            width: 8px;
        }

        ::-webkit-scrollbar-track {
            background: var(--bg-primary);
        }

        ::-webkit-scrollbar-thumb {
            background: var(--border-color);
            border-radius: 4px;
        }

        ::-webkit-scrollbar-thumb:hover {
            background: var(--text-secondary);
        }
    </style>
</head>
<body>
    <div class="container">
        <!-- File Management Section -->
        <div class="file-section">
            <h1>File Management</h1>
            
            <!-- Upload Form -->
            <div class="upload-form">
                <h2>Upload New File</h2>
                <form action="upload.php" method="post" enctype="multipart/form-data">
                    <input type="file" name="fileToUpload" required>
                    <input type="submit" value="Upload File" name="submit">
                </form>
                
                <?php
                if (isset($_SESSION['message'])) {
                    echo '<div class="' . $_SESSION['message_type'] . '">' . $_SESSION['message'] . '</div>';
                    unset($_SESSION['message']);
                    unset($_SESSION['message_type']);
                }
                ?>
            </div>

            <!-- File List -->
            <div class="file-list">
                <h2>Uploaded Files</h2>
                <?php if (empty($files)): ?>
                    <p>No files uploaded yet.</p>
                <?php else: ?>
                    <?php foreach ($files as $file): ?>
                        <div class="file-item">
                            <span><?php echo htmlspecialchars($file); ?></span>
                            <a href="?download=<?php echo urlencode($file); ?>">Download</a>
                        </div>
                    <?php endforeach; ?>
                <?php endif; ?>
            </div>
        </div>

        <!-- Chat Section -->
        <div class="chat-section">
            <h1>Chat</h1>
            
            <div class="chat-messages" id="chatMessages">
                <?php 
                $messages = file_exists($chatFile) ? file_get_contents($chatFile) : '';
                $messageLines = array_filter(explode("\n---\n", $messages));
                foreach ($messageLines as $line): 
                    $line = trim($line);
                    if (empty($line)) continue;
                    
                    $parts = explode(': ', $line, 2);
                    $timestamp = $parts[0];
                    $messageContent = isset($parts[1]) ? $parts[1] : '';
                ?>
                    <div class="message-line">
                        <div class="message-content">
                            <span class="timestamp"><?php echo htmlspecialchars($timestamp); ?>: </span>
                            <span class="text"><?php echo nl2br(htmlspecialchars($messageContent)); ?></span>
                        </div>
                        <button class="copy-btn" title="Copy message">
                            <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                                <rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect>
                                <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path>
                            </svg>
                        </button>
                    </div>
                <?php endforeach; ?>
            </div>
            
            <form class="chat-form" id="chatForm">
                <textarea name="message" placeholder="Type your message..." required></textarea>
                <button type="submit">Send</button>
            </form>
        </div>
    </div>

    <script>
        // Function to copy message to clipboard
        document.addEventListener('click', function(e) {
            if (e.target.closest('.copy-btn')) {
                const button = e.target.closest('.copy-btn');
                const messageText = button.parentElement.querySelector('.text').textContent;
                
                // Create temporary textarea for copying
                const textarea = document.createElement('textarea');
                textarea.value = messageText;
                document.body.appendChild(textarea);
                textarea.select();
                
                try {
                    document.execCommand('copy');
                    // Visual feedback for copy
                    button.style.color = '#4caf50';
                    setTimeout(() => {
                        button.style.color = '';
                    }, 1000);
                } catch (err) {
                    console.error('Failed to copy text: ', err);
                } finally {
                    document.body.removeChild(textarea);
                }
            }
        });

        // Function to update chat messages
        function updateChat() {
            let currentScroll = chatMessages.scrollTop;
            let isScrolledToBottom = chatMessages.scrollHeight - chatMessages.clientHeight <= chatMessages.scrollTop + 1;
            
            fetch('index.php?get_messages')
                .then(response => response.text())
                .then(html => {
                    document.getElementById('chatMessages').innerHTML = html;
                    // Maintain scroll position
                    if (isScrolledToBottom) {
                        chatMessages.scrollTop = chatMessages.scrollHeight;
                    } else {
                        chatMessages.scrollTop = currentScroll;
                    }
                });
        }

        // Handle chat form submission
        document.getElementById('chatForm').addEventListener('submit', function(e) {
            e.preventDefault();
            
            const formData = new FormData(this);
            formData.append('ajax', 'true');

            fetch('index.php', {
                method: 'POST',
                body: formData
            })
            .then(() => {
                this.reset();
                updateChat();
                // Scroll to bottom only after sending a new message
                chatMessages.scrollTop = chatMessages.scrollHeight;
            });
        });
        
        // Initial scroll to bottom only on first load
        const chatMessages = document.getElementById('chatMessages');
        chatMessages.scrollTop = chatMessages.scrollHeight;

        // Handle textarea enter key
        document.querySelector('textarea[name="message"]').addEventListener('keydown', function(e) {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                document.getElementById('chatForm').dispatchEvent(new Event('submit'));
            }
        });
    </script>
</body>
</html>