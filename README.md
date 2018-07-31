# tf_tree_connetor

## Description

Imagine you want to evaluate e.g. a SLAM-approach using a ground truth. Both of
components provide a transformation via TF2 from a map-frame to a base_link-frame.
This results in two distinct TF-trees. You can not simply give the same name to
the base links because this would break the tree. In some cases it might be
possible to flip one of these transformations, but this might not always
feasable.

<img src="frames_before.svg" alt="drawing" width="500px"/>

This problem can be solved using `tf_tree_connector`. `tf_tree_connector` adds a
static transformation between the two map frames. This is done in a way, that
during the start of `tf_tree_connector` bot base_link-frames concided, so that
the transformations `reference_map -> reference_base_link` and
`reference_map -> base_link` are identical.

<img src="frames_after.svg" alt="drawing" width="500px"/>
