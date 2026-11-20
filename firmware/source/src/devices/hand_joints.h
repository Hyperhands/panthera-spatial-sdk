#pragma once

/* Handle-L1 protocol V1.1, angle feedback (page 18): active thumb rotation,
 * active thumb bend, passive thumb bend, then active/passive pairs per finger.
 * CMD09 accepts only the six active joints, in this order. */
static const unsigned hand_active_feedback_index[6]={0,1,3,5,7,9};
