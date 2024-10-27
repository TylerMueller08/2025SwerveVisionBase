Below is a template that can be plugged into ChatGPT to generate some documentation for a class.

Note that the template does not support subclasses. If a class contains a subclass then, in the Subclasses section, add a link to a separate file which contains the documentation for the subclass. The subclass documentation should follow this template as well. However, if the subclass is very small you can add the information in the Subclass section, attempting to follow the template format as applies.

The current version of this template has limited support for enums. If you are using a fancy enum then make sure to add relevant extra information. Be sure to proofread the documentation before publishing it. This template should be followed even if you are not using ChatGPT.

Here is the AI prompt:

Hello ChatGPT. I would like you to generate documentation for this java class. It is built on wpilib and photonvision. I would like you to use the template below to generate the documentation. When applying the template, things in curly braces should be removed and things in square brackets should be substituted with appropriate values. Here is the template:

# [Class Name]
Class package: [the package that the class is in]

## Purpose
[Explain the purpose of the class here]

## Public Fields:
{Describe the different public fields that the class contains, if any. Only document the public fields. If there are no public fields do not add this section. The public fields should be in list format, with one field per list bullet point. It should follow the same pattern as this list follows. The list created should have as many bullet points as there are public fields.}
-  `[Name of 1st field]`: [Description of the 1st field's type and purpose]

-  `[Name of 2nd field]`: [Description of the 2nd field's type and purpose]

-  `[Name of 3rd field]`: [Description of the 3rd field's type and purpose]

## Public Methods
{Describe the different public methods that the class contains, if any. Only document the public methods. If there are no public methods do not add this section. Add one subheading per public method. There should be as many subheadings as there are public methods. Include a --- between each public method}

### `[Name of 1st method, e.g. public static ReturnType exampleMethod(parameter1, parameter2)]`
- **Description:** [A description of what the method does]
- **Parameters**: {Include as many entries in this list as there are parameters}
  - [A description of parameter 1, including its type and how it is used]
  - [A description of parameter 2, including its type and how it is used]
- **Returns**: [the return type and a description of what the return value means. Do not include this section if the return type is void]

---
### `[Name of 2nd method, e.g. public void exampleMethod2()]`
* **Description**: [A description of what the function does]

## Subclasses

{Only include this section if the class contains one or more subclasses. If there are no subclasses do not include this section.}
Placeholder for `[SubclassName1]`.
Placeholder for `[SubclassName2]`.
The documentation for these subclasses will be added by humans.

## Enums
{Only include this section if the class contains one or more enums. If there are no enums do not include this section.}

### `[Enum Name]`
{Include the different possible values of the enum in a list. There should be one list item per enum value. The number of list items should be the same as the number of enum values.}
- **Description**: [Description of what the enum is used for.]
- **Values**:
  -  `[Name of 1st value]`: [Description of the 1st value's meaning]
  -  `[Name of 2nd value]`: [Description of the 2nd value's meaning]
  -  `[Name of 3rd value]`: [Description of the 3rd value's meaning]