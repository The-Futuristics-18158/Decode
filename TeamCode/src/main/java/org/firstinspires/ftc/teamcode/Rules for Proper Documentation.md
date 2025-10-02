**Rules for Proper Documentation**

* Make file, class, function, parameter, and variable names descriptive and relevant.
* Use camel case when naming class, function, parameter, and variable (ie. thisIsCamelCase  this\_is\_not\_camel\_case)
* File names should be done in camel case with the first letter also capitalized (ie. RedAuto)
* ADD COMMENTS. Just because you know how a chunk of code works doesn't mean other team members will.


**When doing Javadoc you must include**

* a description of the function/class
* your GitHub username
* a description of each parameter(if applicable)
* a description of what is returned(if applicable)

**Example:**

/**
 * This method takes in two numbers and adds them.
 * 
 * @author Kw126
 * 
 * @param num1 the first number being added
 * @param num2 the second number being added
 * 
 * @return the result of the addition
 */
public static int addition(int num1, int num2){

    //Initialize Variables
    int result;
    
    //Adds the two numbers together
    result = num1 + num2;
    
    return result;
}

