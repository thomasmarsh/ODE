package.name = "Ode.NET"
package.kind = "dll"
package.language = "c#"

if (options["with-doubles"]) then
  package.defines = { "dDOUBLE" }
else
  package.defines = { "dSINGLE " }
end

package.links = {
  "System"
}

package.files = {
  "AssemblyInfo.cs",
  "Ode.cs"
}
