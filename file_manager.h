#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <Arduino.h>
#include <SD.h>
#include "types.h"
#include "config.h"

// ================ FILE MANAGER GLOBALS ================
extern char currentFilePath[128];
extern unsigned long currentFileSize;
extern RotateReason_t lastRotateReason;

// ================ FUNCTION PROTOTYPES ================
void initFileManager();
const char* getFileTypePrefix(FileType_t type);
void generateFilePath(char* buffer, size_t len, FileType_t fileType, 
                      uint32_t sessionId, uint32_t fileSeq, const char* suffix);
void createNewLogFile();
bool needsFileRotation();
void rotateFile(RotateReason_t reason);
void closeCurrentFile(RotateReason_t reason);
void deleteFile(const char* fileName);
void listFiles();
void listDiagFiles();
void listFilesByType(File dir, const char* prefix);
String findFileRecursive(const char* basePath, const char* targetFile);
void listDirContents(File dir, int level);

// Diagnostic file functions
void createDisconnectDiagFile(RotateReason_t reason, uint32_t lastRecErr, uint32_t lastTecErr);
void createErrorDiagFile(const char* errorType, uint32_t errorCode);
void createRecoveryDiagFile();
void listDiagFilesRecursive(File dir, int level);

// File transfer functions
void sendFile(const char* fileName);
void sendFileNormal(const char* fullPath, const char* displayName);
bool sendFileCompressed(const char* fullPath, const char* displayName);
void sendDataInChunks(uint8_t* data, size_t dataSize);
size_t simpleRLECompress(const uint8_t* input, size_t inputSize, 
                         uint8_t* output, size_t outputSize);
// Add these to the function prototypes section
void createTestFile(const char* fileName);
void deleteFile(const char* fileName);
void listLogsByDate(int year, int month, int day);


void saveConfigToSPIFFS();
void loadConfigFromSPIFFS();

#endif