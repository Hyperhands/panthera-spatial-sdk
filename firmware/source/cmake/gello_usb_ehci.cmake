# The MCUX transaction-done walk races with cancellation from an application
# thread. Both touch the same QH/transfer list, but only cancellation takes
# ehciMutex. Keep the fix local to this application, without changing the SDK.
get_target_property(gello_hal_sources modules__hal_nxp SOURCES)
set(gello_ehci_original "")
foreach(gello_hal_source IN LISTS gello_hal_sources)
  if(gello_hal_source MATCHES "/usb_host_ehci\\.c$")
    set(gello_ehci_original "${gello_hal_source}")
  endif()
endforeach()
if(NOT gello_ehci_original OR NOT EXISTS "${gello_ehci_original}")
  message(FATAL_ERROR "Cannot locate MCUX EHCI source for the checked cancellation fix")
endif()
file(READ "${gello_ehci_original}" gello_ehci_source)
set(gello_ehci_old "                USB_HostEhciTransactionDone(ehciInstance);")
string(FIND "${gello_ehci_source}" "${gello_ehci_old}" gello_ehci_pos)
if(gello_ehci_pos LESS 0)
  message(FATAL_ERROR "MCUX EHCI source changed; review transaction/cancellation locking")
endif()
string(REPLACE "${gello_ehci_old}"
  "                /* Serialize QH traversal with cancellation/pipe teardown. */\n                USB_HostEhciLock();\n                USB_HostEhciTransactionDone(ehciInstance);\n                USB_HostEhciUnlock();"
  gello_ehci_source "${gello_ehci_source}")
set(gello_ehci_fixed "${CMAKE_CURRENT_BINARY_DIR}/usb_host_ehci_gello.c")
file(WRITE "${gello_ehci_fixed}" "${gello_ehci_source}")
list(REMOVE_ITEM gello_hal_sources "${gello_ehci_original}")
set_property(TARGET modules__hal_nxp PROPERTY SOURCES "${gello_hal_sources}")
target_sources(modules__hal_nxp PRIVATE "${gello_ehci_fixed}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${gello_ehci_original}")
