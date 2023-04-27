# Instructions

- Fork the CadQuery repository, clone your fork and create a new branch to start working on your changes
- Create a conda development environment with something like: 
`conda env create -n cq-cam-dev -f environment.yml`
- Activate the new conda environment: 
conda activate cq-cam-dev
- Install `cq-cam`:
`pip install -e .`
- Before making any changes verify that the current tests pass. Run pytest from the root of your cq-cam clone, there should be no failures and the output will look similar to this:
- Start with the tests! How should CadQuery behave after your changes? Make sure to add some tests to the test suite to ensure proper behavior
- Make sure your tests have assertions checking all the expected results
- Add a nice docstring to the test indicating what the test is doing; if there is too much to explain, consider splitting the test in two!
- Go ahead and implement the changes
- Add a nice docstring to the functions/methods/classes you implement describing what they do, what the expected parameters are and what it returns (if anything)
- Update the documentation if there is any change to the public API
- Consider adding an example to the documentation showing your cool new feature!
- Make sure nothing is broken (run the complete test suite with pytest)
- Run black to autoformat your code and make sure your code style complies with cq-cam's
- Push the changes to your fork and open a pull-request upstream
- Keep an eye on the automated feedback you will receive from the CI pipelines; if there is a test failing or some code is not properly formatted, you will be notified without human intervention
- Be prepared for constructive feedback and criticism!
- Be patient and respectful, remember that those reviewing your code are also working hard (sometimes reviewing changes is harder than implementing them!)