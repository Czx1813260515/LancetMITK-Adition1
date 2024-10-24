project(org_mitk_lancet_InstantiationCamera)

mitk_create_plugin(
  EXPORT_DIRECTIVE INSTANTIATIONCAMERA_EXPORT
  EXPORTED_INCLUDE_SUFFIXES src
  PACKAGE_DEPENDS Qt5|Widgets+Multimedia OpenCV
  MODULE_DEPENDS MitkQtWidgetsExt MitkLancetHardwareDevice MitkLancetFileIO
)
