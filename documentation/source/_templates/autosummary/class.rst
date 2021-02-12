{{ objname | escape | underline}}

.. currentmodule:: {{ module }}

.. autoclass:: {{ objname }}

   {% block attributes %}
   {% if attributes %}
   .. rubric:: Attributes
   .. autosummary::
      :toctree:
   {% for item in attributes %}
      {{ name }}.{{ item }}
   {% endfor %}
   {% endif %}
   {% endblock %}

   {% block methods %}
   {% if methods %}
   .. rubric:: Methods
   .. autosummary::
      :toctree:
      :nosignatures:
   {% for item in methods %}
    {%- if not item.startswith('__') %}
      {{ name }}.{{ item }}
    {%- endif -%}
   {% endfor %}
   {% endif %}
   {% endblock %}


This page is auto-generated. Page source is not available on Github.
