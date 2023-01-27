# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'TigerASI'
copyright = '2023, poofjunior, adamkglaser'
author = 'poofjunior, adamkglaser'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.napoleon",  # enable numpy and google style docstring parsing.
    "sphinx.ext.autodoc",  # enable doc generation from code docstrings.
]

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'furo'
#html_theme = 'classic'
#html_theme = 'alabaster'
html_theme_options = {
    #'page_width': 'auto',  # alabaseter theme spec
    #'fixed_sidebar': True,  # alabaster theme spec
    #'sidebar_width': '320px',  # alabaster theme spec
#    'body_min_width': '70%', # classic theme spec
#    'sidebarwidth': '320px', # classic theme spec
}
# 'classic' theme sidebar settings.
#html_sidebars = {'**': ['localtoc.html', 'sourcelink.html',
#                        'searchbox.html', 'globaltoc.html']}
html_static_path = []  #['_static']

# Sphinx Options.
autoclass_content = 'both'
toc_object_entries_show_parents = 'hide' # hide the full method path.
