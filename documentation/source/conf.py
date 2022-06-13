# Configuration file for the Sphinx documentation builder.

# -- Project information
import m2r2
project = 'open3d_slam'
copyright = '2022, open3d_slam authors'
author = 'see github repository'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'm2r2',
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'
source_suffix = [".rst", ".md"]
html_extra_path = ['extra']

# -- Options for EPUB output
epub_show_urls = 'footnote'
