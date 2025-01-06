<?php
session_start();
error_reporting(E_ALL);
ini_set('display_errors', 1);

// Configuration
$uploadDir = 'uploads/';

// Function to convert PHP size string to bytes
function convertPHPSizeToBytes($sSize) {
    $sSuffix = strtoupper(substr($sSize, -1));
    if (!in_array($sSuffix, array('P','T','G','M','K'))){
        return (int)$sSize;
    }
    $iValue = substr($sSize, 0, -1);
    switch ($sSuffix) {
        case 'P':
            $iValue *= 1024;
        case 'T':
            $iValue *= 1024;
        case 'G':
            $iValue *= 1024;
        case 'M':
            $iValue *= 1024;
        case 'K':
            $iValue *= 1024;
            break;
    }
    return (int)$iValue;
}

// Get actual max upload size from PHP configuration
$maxUpload = min(
    convertPHPSizeToBytes(ini_get('upload_max_filesize')),
    convertPHPSizeToBytes(ini_get('post_max_size'))
);

// Create uploads directory if it doesn't exist
if (!file_exists($uploadDir)) {
    mkdir($uploadDir, 0755, true);
}

// Check if directory is writable
if (!is_writable($uploadDir)) {
    $_SESSION['message'] = "Upload directory is not writable. Please check permissions.";
    $_SESSION['message_type'] = "error";
    header("Location: index.php");
    exit;
}

if ($_SERVER["REQUEST_METHOD"] == "POST" && isset($_FILES["fileToUpload"])) {
    $file = $_FILES["fileToUpload"];
    
    // Detailed error checking
    switch ($file["error"]) {
        case UPLOAD_ERR_OK:
            break;
        case UPLOAD_ERR_INI_SIZE:
            $_SESSION['message'] = "The uploaded file exceeds the upload_max_filesize directive (" . ini_get('upload_max_filesize') . ") in php.ini";
            $_SESSION['message_type'] = "error";
            header("Location: index.php");
            exit;
        case UPLOAD_ERR_FORM_SIZE:
            $_SESSION['message'] = "The uploaded file exceeds the MAX_FILE_SIZE directive that was specified in the HTML form";
            $_SESSION['message_type'] = "error";
            header("Location: index.php");
            exit;
        case UPLOAD_ERR_PARTIAL:
            $_SESSION['message'] = "The uploaded file was only partially uploaded";
            $_SESSION['message_type'] = "error";
            header("Location: index.php");
            exit;
        case UPLOAD_ERR_NO_FILE:
            $_SESSION['message'] = "No file was uploaded";
            $_SESSION['message_type'] = "error";
            header("Location: index.php");
            exit;
        case UPLOAD_ERR_NO_TMP_DIR:
            $_SESSION['message'] = "Missing a temporary folder";
            $_SESSION['message_type'] = "error";
            header("Location: index.php");
            exit;
        case UPLOAD_ERR_CANT_WRITE:
            $_SESSION['message'] = "Failed to write file to disk";
            $_SESSION['message_type'] = "error";
            header("Location: index.php");
            exit;
        case UPLOAD_ERR_EXTENSION:
            $_SESSION['message'] = "File upload stopped by extension";
            $_SESSION['message_type'] = "error";
            header("Location: index.php");
            exit;
        default:
            $_SESSION['message'] = "Unknown upload error";
            $_SESSION['message_type'] = "error";
            header("Location: index.php");
            exit;
    }

    // Add this for debugging
    error_log("File size: " . $file["size"] . " bytes");
    error_log("Max upload size: " . $maxUpload . " bytes");
    error_log("Upload max filesize: " . ini_get('upload_max_filesize'));
    error_log("Post max size: " . ini_get('post_max_size'));
    
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
    
    // Check disk space
    $freeSpace = disk_free_space($uploadDir);
    if ($freeSpace !== false && $file["size"] > $freeSpace) {
        $_SESSION['message'] = "Not enough disk space to upload this file";
        $_SESSION['message_type'] = "error";
        header("Location: index.php");
        exit;
    }
    
    // Try to move the uploaded file
    if (move_uploaded_file($file["tmp_name"], $targetPath)) {
        $_SESSION['message'] = "File uploaded successfully!";
        $_SESSION['message_type'] = "success";
    } else {
        $error = error_get_last();
        $_SESSION['message'] = "Error uploading file: " . ($error['message'] ?? 'Unknown error');
        $_SESSION['message_type'] = "error";
        error_log("Upload error: " . print_r($error, true));
    }
} else {
    $_SESSION['message'] = "No file selected or invalid request";
    $_SESSION['message_type'] = "error";
}

header("Location: index.php");
exit;