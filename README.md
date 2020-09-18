![Documentation Status](https://readthedocs.org/projects/studica-docs/badge/?version=latest)
# studica-docs

The Studica Documentation is available in these domains:
- https://docs.studica.com/en/latest
- https://studica-docs.readthedocs.io/en/latest

After pushing an update to the repo the readthedocs plugin will build the project and publish. Note it it may take up to 10 min before the changes are shown!

## Building Locally

It is recommended to run the changes made locally before pushing to here for an update. To run the documentation locally python 3.7+ is required. Python 3.7+ can be downloaded [here](https://www.python.org/downloads/). 

After python is downloaded run the following commands in command prompt.

- ``pip install "Sphinx==3.03"``
- ``pip install sphinx-tabs`` 
- ``pip install sphinx-rtd-theme``
- ``pip install sphinx-notfound-page``

Clone the repo using git into your local system. To build the docs open command prompt and navigate to the folder where the Makefile is located. In that folder run ``make html``. This will show a display of the compilation and if completed without errors there will be a new folder called build. To open the local version of the docs, open the index.html file located inside build/html/

**Note** you need to run ``make html`` every time you make a change.

If you get a **app.add_javascript() is deprecated** error follow these steps:

1. Locate the tabs.py folder. Example path ``C:\python\lib\site-packages\sphinx_tabs\``
2. Open tabs.py using a text editor.
3. Navigate to line 341 or search for ``app.add_javascript(path)``
4. Replace ``app.add_javascript(path)`` with ``app.add_js_file(path)``
5. Save and close. 