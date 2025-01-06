<?php
session_start();

// Configuration
$uploadDir = 'uploads/';
$maxFileSize = 5 * 1024 * 1024; // 5MB

// Create uploads directory if it doesn't exist
if (!file_exists($uploadDir)) {
    mkdir($uploadDir, 0755, true);
}

if ($_SERVER["REQUEST_METHOD"] == "POST" && isset($_FILES["fileToUpload"])) {
    $file = $_FILES["fileToUpload"];
    
    // Basic error checking
    if ($file["error"] !== UPLOAD_ERR_OK) {
        $_SESSION['message'] = "Upload failed with error code: " . $file["error"];
        $_SESSION['message_type'] = "error";
        header("Location: index.php");
        exit;
    }
    
    // Size validation
    // if ($file["size"] > $maxFileSize) {
    //     $_SESSION['message'] = "File is too large. Maximum size is 5MB.";
    //     $_SESSION['message_type'] = "error";
    //     header("Location: index.php");
    //     exit;
    // }
    
    // Sanitize filename
    $filename = basename($file["name"]);
    $filename = preg_replace("/[^a-zA-Z0-9._-]/", "", $filename);
    
    // Generate unique filename if file already exists
    $targetPath = $uploadDir . $filename;
    $i = 1;
    while (file_exists($targetPath)) {
        $info = pathinfo($filename);
        $filename = $info['filename'] . '_' . $i . '.' . $info['extension'];
        $targetPath = $uploadDir . $filename;
        $i++;
    }
    
    // Move uploaded file
    if (move_uploaded_file($file["tmp_name"], $targetPath)) {
        $_SESSION['message'] = "File uploaded successfully!";
        $_SESSION['message_type'] = "success";
    } else {
        $_SESSION['message'] = "Error uploading file.";
        $_SESSION['message_type'] = "error";
    }
} else {
    $_SESSION['message'] = "No file selected.";
    $_SESSION['message_type'] = "error";
}

header("Location: index.php");
exit;