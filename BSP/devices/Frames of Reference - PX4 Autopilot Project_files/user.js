/* en: Place for user defined JavaScript - this file can safely be preserved
   when updating. See README for details.
   ATTENTION: Do not forget to activate the template option
              "mnmlblog_loaduserjs" (->"Load 'mnml-blog/user/user.js'?") in the
              DokuWiki Config Manager! Otherwise, any changes to this file
              won't have any effect!
   
   de: Ort für benutzerdefiniertes JavaScript - Diese Datei kann beim
   Durchführen von Updates ohne Risiko beibehalten werden. Konsultieren Sie
   die README für Detailinformationen.

   ACHTUNG: Vergessen Sie nicht die Template-Option "mnmlblog_loaduserjs"
            (->"Datei 'mnml-blog/user/user.js' laden?") im DokuWiki Config
            Manager zu aktivieren! Andernfalls werden sämtliche Änderungen an
            dieser Datei ohne Auswirkungen bleiben! */

var username = document.getElementById('_username').innerHTML;
document.getElementById('_username').innerHTML = "";
var loc = location.href;
var url = loc.substr(0, loc.indexOf('?'));
var anchor = document.getElementById("_login_links");
var newul = document.createElement("ul");


if (username == "") {
var regHTML = "<li class='level1'><div class='li'><a href='"+url+"?do=register'>Register</a></div></li>"
var loginHTML = "<li class='level1'><div class='li'><a href='"+url+"?do=login'>Login to Edit</a></div></li>"
newul.innerHTML = regHTML + loginHTML;
} else {
newul.innerHTML = "<li class='level1'><div class='li'><a href='"+url+"?do=edit'>Edit this Page</a></div></li>"
}
anchor.parentNode.replaceChild(newul, anchor);

  (function(i,s,o,g,r,a,m){i['GoogleAnalyticsObject']=r;i[r]=i[r]||function(){
  (i[r].q=i[r].q||[]).push(arguments)},i[r].l=1*new Date();a=s.createElement(o),
  m=s.getElementsByTagName(o)[0];a.async=1;a.src=g;m.parentNode.insertBefore(a,m)
  })(window,document,'script','//www.google-analytics.com/analytics.js','ga');

  ga('create', 'UA-33658859-2', 'pixhawk.org');
  ga('send', 'pageview');
