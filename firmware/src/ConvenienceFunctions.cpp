
#include "ConvenienceFunctions.h"

using namespace project_namespace;

void project_namespace::serialPrintMatrix(const Eigen::MatrixXd& matrix) {
    /*
    DESCRIPTION:
    Formats an Eigen matrix and sends it over through a series of Serial.print().
    Prints the number of rows, number of columns and the matrix itself.
    This takes in a matrix of integers.
    
    ARGUMENTS:
    + matrix: The matrix to print.
    */
    int i, j, num_of_rows, num_of_cols;
    
    num_of_rows = matrix.rows();
    num_of_cols = matrix.cols();

    Serial.print("num_of_rows: "); Serial.println(num_of_rows);
    Serial.print("num_of_cols: "); Serial.println(num_of_cols);      
    Serial.println();
    
    for (i = 0; i < num_of_rows; i++)
    {
        for (j = 0; j < num_of_cols; j++)
        {
            Serial.print(matrix(i,j), 6);   // print 6 decimal places
            Serial.print(", ");
        }
        Serial.println();
    }
    Serial.println();
}

void project_namespace::serialPrintMatrix(const Eigen::MatrixXf& matrix) {
    /*
    DESCRIPTION:
    Formats an Eigen matrix and sends it over through a series of Serial.print().
    Prints the number of rows, number of columns and the matrix itself.
    This takes in a matrix of floats.
    
    ARGUMENTS:
    + matrix: The matrix to print.
    */
    int i, j, num_of_rows, num_of_cols;
    
    num_of_rows = matrix.rows();
    num_of_cols = matrix.cols();

    Serial.print("num_of_rows: "); Serial.println(num_of_rows);
    Serial.print("num_of_cols: "); Serial.println(num_of_cols);      
    Serial.println();
    
    for (i = 0; i < num_of_rows; i++)
    {
        for (j = 0; j < num_of_cols; j++)
        {
            Serial.print(matrix(i,j), 6);   // print 6 decimal places
            Serial.print(", ");
        }
        Serial.println();
    }
    Serial.println();
}