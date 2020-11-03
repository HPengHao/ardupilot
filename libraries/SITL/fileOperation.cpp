#include "fileOperation.h"
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

void readCSV(string filename, vector<vector<double>>& outcome){
    // File pointer 
    fstream fin; 
  
    // Open an existing file 
    fin.open(filename, ios::in); 

    // Make sure the file is open
    if(!fin.is_open()){
        cout << "Could not open csv file: " << filename << endl;
        return;
    }

    // Helper vars
    string line, colname;
    double val; 
    // Read the column names
    if(fin.good())
    {
        // Extract the first line in the file
        getline(fin, line);

        // Create a stringstream from line
        stringstream ss(line);

        // Extract each column name
        while(getline(ss, colname, ',')){
            // Initialize and add <colname, int vector> pairs to result
            cout << colname << ", ";
        }
        cout<<endl;
        
        // Read data, line by line
        while(getline(fin, line))
        {
            // Create a stringstream of the current line
            ss = stringstream(line);
            
            // Keep track of the current column index
            int colIdx = 0;
            vector<double> lineArr;
            // Extract each data
            while(ss >> val){
                lineArr.push_back(val);
                // If the next token is a comma, ignore it and move on
                if(ss.peek() == ',') ss.ignore();
                // Increment the column index
                colIdx++;
            }
            outcome.push_back(lineArr);

        }
    }
    fin.close();
}