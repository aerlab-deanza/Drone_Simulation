{{ fullname | escape | underline }}

.. automodule:: {{ fullname }}
   :no-members:

{% block modules %}
{% if modules %}
.. rubric:: Submodules

.. autosummary::
   :toctree:
   :template: autosummary/module.rst
   :recursive:
{% for item in modules %}
   {{ item }}
{%- endfor %}
{% endif %}
{% endblock %}

{% if classes %}
.. rubric:: Classes

.. autosummary::
   :nosignatures:
{% for item in classes %}
   {{ item }}
{%- endfor %}
{% endif %}

{% if functions %}
.. rubric:: Functions

.. autosummary::
   :nosignatures:
{% for item in functions %}
   {{ item }}
{%- endfor %}
{% endif %}

{% if attributes %}
.. rubric:: Constants

.. autosummary::
{% for item in attributes %}
   {{ item }}
{%- endfor %}
{% endif %}

{% if attributes or classes or functions %}
Reference
---------
{% endif %}

{% for item in attributes %}
.. autodata:: {{ item }}
{% endfor %}

{% for item in classes %}
.. autoclass:: {{ item }}
   :members:
   :show-inheritance:
{% endfor %}

{% for item in functions %}
.. autofunction:: {{ item }}
{% endfor %}
