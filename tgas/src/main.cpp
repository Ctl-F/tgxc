//
// Created by ctlf on 1/24/25.
//
#include "TGX.h"

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unistd.h>
#include <sys/wait.h>
#include <stdexcept>
#include <fstream>

std::string preprocess_input(const char* filename);

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Please provide a source file.\n";
        return 1;
    }

    std::string output;

    try {
        output = preprocess_input(argv[1]);
    }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n";
        return 1;
    }

    // preprocessing is working, need to filter out preprocessor comments #...
    // then we need to actually tokenize and assemble the output.
    // this should be pretty straightforward, seeing as the preprocessor is done already.

    return 0;
}

std::string preprocess_input(const char *filename) {
    std::vector<char> bytes;
    int pipefd[2];
    pid_t pid;

    // preprocess file
    if (pipe(pipefd) == -1) {
        perror("pipe");
        throw std::runtime_error("Could not create pipe for preprocessing");
    }

    pid = fork();
    if (pid == -1) {
        perror("fork");
        throw std::runtime_error("Error forking the current process.");
    }

    if (pid == 0) {
        // child process
        close(pipefd[0]);
        if (dup2(pipefd[1], STDOUT_FILENO) == -1) {
            perror("dup2"); // dup2 redirects stdout to this pipe
            exit(EXIT_FAILURE);
        }
        close(pipefd[1]);

        execlp("cc", "cc", "-E", "-x", "c", filename, NULL);
        //execlp only returns on failure
        perror("execlp");
        exit(EXIT_FAILURE);
    }

    // parent process
    close(pipefd[1]);

    char buffer[1024];
    ssize_t bytes_read;

    while ((bytes_read = read(pipefd[0], buffer, sizeof(buffer)-1)) > 0) {
        bytes.insert(bytes.end(), buffer, buffer+bytes_read);
    }

    if (bytes_read == -1) {
        perror("read");
        close(pipefd[0]);
        throw std::runtime_error("Unable to read from output");
    }

    close(pipefd[0]);
    int status;
    waitpid(pid, &status, 0);

    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        throw std::runtime_error("Preprocessing failed with an error.");
    }

    return { bytes.begin(), bytes.end() };
}
